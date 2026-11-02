//! Plasma CAM: parts from DXF outlines (lines and true arcs), true-shape sheet nesting, a SheetCAM-compatible
//! tool library, cut-time quoting and QtPlasmaC G-code with G2/G3 arcs.
//! Pure data and arithmetic; the workspace page (`plasma_cam.rs`) draws and edits it.
//! All lengths are millimetres and feeds mm/min unless a name says otherwise.

use std::f64::consts::TAU;

use solidcraft_engine::io::Geom2;

pub const MM_PER_IN: f64 = 25.4;
/// Endpoints closer than this join into one chain.
const JOIN_TOL: f64 = 0.05;
/// Chord step when arcs become polylines for drawing and nesting.
const ARC_STEP_DEG: f64 = 5.0;
/// Most part instances one nest places (hostile quantities).
pub const MAX_INSTANCES: usize = 5000;

pub type P2 = [f64; 2];

fn dist(a: P2, b: P2) -> f64 {
    ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2)).sqrt()
}
fn lerp(a: P2, b: P2, t: f64) -> P2 {
    [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t]
}

// --- Geometry: contours of lines and arcs ------------------------------------------------------

/// One step of a path, from the previous point.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Seg {
    Line(P2),
    /// Circular arc to `to` around `c`, counter-clockwise when `ccw`.
    Arc {
        to: P2,
        c: P2,
        ccw: bool,
    },
}

impl Seg {
    pub fn end(&self) -> P2 {
        match *self {
            Seg::Line(p) | Seg::Arc { to: p, .. } => p,
        }
    }
    fn map(&self, f: &dyn Fn(P2) -> P2) -> Seg {
        match *self {
            Seg::Line(p) => Seg::Line(f(p)),
            Seg::Arc { to, c, ccw } => Seg::Arc { to: f(to), c: f(c), ccw },
        }
    }
}

/// Swept angle of an arc from `from` (always positive, up to a full turn).
fn sweep(from: P2, to: P2, c: P2, ccw: bool) -> f64 {
    let a0 = (from[1] - c[1]).atan2(from[0] - c[0]);
    let a1 = (to[1] - c[1]).atan2(to[0] - c[0]);
    let mut s = if ccw { a1 - a0 } else { a0 - a1 };
    while s <= 1e-9 {
        s += TAU;
    }
    while s > TAU + 1e-9 {
        s -= TAU;
    }
    s
}

fn arc_points(from: P2, to: P2, c: P2, ccw: bool, out: &mut Vec<P2>) {
    let r = dist(from, c);
    let s = sweep(from, to, c, ccw);
    let a0 = (from[1] - c[1]).atan2(from[0] - c[0]);
    let n = ((s.to_degrees() / ARC_STEP_DEG).ceil() as usize).clamp(1, 720);
    for i in 1..n {
        let a = a0 + if ccw { 1.0 } else { -1.0 } * s * i as f64 / n as f64;
        out.push([c[0] + r * a.cos(), c[1] + r * a.sin()]);
    }
    out.push(to);
}

/// A closed outline: the outer profile or a hole. The last segment ends at `start`.
#[derive(Clone, Debug, PartialEq)]
pub struct Contour {
    pub start: P2,
    pub segs: Vec<Seg>,
    /// Polyline for drawing and nesting (without the closing point).
    pub pts: Vec<P2>,
}

impl Contour {
    pub fn new(start: P2, segs: Vec<Seg>) -> Self {
        let mut c = Contour { start, segs, pts: Vec::new() };
        c.tessellate();
        c
    }
    fn from_points(pts: Vec<P2>) -> Self {
        let start = pts.first().copied().unwrap_or([0.0, 0.0]);
        let segs = pts.iter().skip(1).chain(std::iter::once(&start)).map(|p| Seg::Line(*p)).collect();
        Contour::new(start, segs)
    }
    fn circle(c: P2, r: f64) -> Self {
        let a = [c[0] + r, c[1]];
        let b = [c[0] - r, c[1]];
        Contour::new(a, vec![Seg::Arc { to: b, c, ccw: true }, Seg::Arc { to: a, c, ccw: true }])
    }
    fn tessellate(&mut self) {
        let mut pts = vec![self.start];
        let mut at = self.start;
        for s in &self.segs {
            match *s {
                Seg::Line(p) => pts.push(p),
                Seg::Arc { to, c, ccw } => arc_points(at, to, c, ccw, &mut pts),
            }
            at = s.end();
        }
        if pts.len() > 1 && pts.last().is_some_and(|p| dist(*p, self.start) < 1e-6) {
            pts.pop();
        }
        self.pts = pts;
    }
    /// Signed area (positive = counter-clockwise).
    pub fn signed_area(&self) -> f64 {
        let n = self.pts.len();
        (0..n).filter_map(|i| Some((*self.pts.get(i)?, *self.pts.get((i + 1) % n)?))).map(|(p, q)| p[0] * q[1] - q[0] * p[1]).sum::<f64>() / 2.0
    }
    /// Exact length, arcs included.
    pub fn length(&self) -> f64 {
        let mut at = self.start;
        let mut l = 0.0;
        for s in &self.segs {
            l += match *s {
                Seg::Line(p) => dist(at, p),
                Seg::Arc { to, c, ccw } => dist(at, c) * sweep(at, to, c, ccw),
            };
            at = s.end();
        }
        l
    }
    pub fn bounds(&self) -> (P2, P2) {
        let mut lo = [f64::INFINITY; 2];
        let mut hi = [f64::NEG_INFINITY; 2];
        for p in &self.pts {
            lo = [lo[0].min(p[0]), lo[1].min(p[1])];
            hi = [hi[0].max(p[0]), hi[1].max(p[1])];
        }
        (lo, hi)
    }
    pub fn reversed(&self) -> Contour {
        let mut pts = vec![self.start];
        pts.extend(self.segs.iter().map(Seg::end));
        // pts[i] → pts[i+1] by segs[i]; walk back.
        let mut segs = Vec::with_capacity(self.segs.len());
        for (i, s) in self.segs.iter().enumerate().rev() {
            let to = pts.get(i).copied().unwrap_or(self.start);
            segs.push(match *s {
                Seg::Line(_) => Seg::Line(to),
                Seg::Arc { c, ccw, .. } => Seg::Arc { to, c, ccw: !ccw },
            });
        }
        Contour::new(self.start, segs)
    }
    pub fn map(&self, f: &dyn Fn(P2) -> P2) -> Contour {
        let segs = self.segs.iter().map(|s| s.map(f)).collect();
        let mut c = Contour { start: f(self.start), segs, pts: self.pts.iter().map(|p| f(*p)).collect() };
        if c.pts.is_empty() {
            c.tessellate();
        }
        c
    }
    /// The point and travel direction halfway along segment `i`.
    fn mid_of(&self, i: usize) -> Option<(P2, P2)> {
        let from = if i == 0 { self.start } else { self.segs.get(i - 1)?.end() };
        Some(match *self.segs.get(i)? {
            Seg::Line(p) => {
                let l = dist(from, p).max(1e-12);
                (lerp(from, p, 0.5), [(p[0] - from[0]) / l, (p[1] - from[1]) / l])
            }
            Seg::Arc { to, c, ccw } => {
                let r = dist(from, c);
                let a = (from[1] - c[1]).atan2(from[0] - c[0]) + if ccw { 0.5 } else { -0.5 } * sweep(from, to, c, ccw);
                let p = [c[0] + r * a.cos(), c[1] + r * a.sin()];
                let t = if ccw { [-a.sin(), a.cos()] } else { [a.sin(), -a.cos()] };
                (p, t)
            }
        })
    }
    fn seg_len(&self, i: usize) -> f64 {
        let from = if i == 0 { self.start } else { self.segs.get(i.wrapping_sub(1)).map_or(self.start, Seg::end) };
        match self.segs.get(i) {
            Some(Seg::Line(p)) => dist(from, *p),
            Some(Seg::Arc { to, c, ccw }) => dist(from, *c) * sweep(from, *to, *c, *ccw),
            None => 0.0,
        }
    }
    /// The same loop starting halfway along segment `i`, the segment split in two.
    pub fn restart_at_mid(&self, i: usize) -> Contour {
        let Some((mid, _)) = self.mid_of(i) else { return self.clone() };
        let Some(seg) = self.segs.get(i) else { return self.clone() };
        let (first_half, second_half) = match *seg {
            Seg::Line(p) => (Seg::Line(mid), Seg::Line(p)),
            Seg::Arc { to, c, ccw } => (Seg::Arc { to: mid, c, ccw }, Seg::Arc { to, c, ccw }),
        };
        let mut segs = vec![second_half];
        segs.extend(self.segs.iter().skip(i + 1).copied());
        segs.extend(self.segs.iter().take(i).copied());
        segs.push(first_half);
        Contour::new(mid, segs)
    }
}

/// A flat part ready to nest: its outer profile first (clockwise), then holes (counter-clockwise), placed with
/// its bounding box at the origin.
#[derive(Clone, Debug)]
pub struct Part {
    pub name: String,
    pub contours: Vec<Contour>,
    pub width: f64,
    pub height: f64,
    pub qty: u32,
    /// Open chains the DXF had that could not be closed (not cut).
    pub open_chains: usize,
    /// Kerf side (SheetCAM's contour methods).
    pub side: Side,
    /// Cut every contour the other way round (moves the square edge to the other side).
    pub reverse: bool,
    /// With no offset: lead in and out from the other side of the line.
    pub flip_lead: bool,
}

impl Part {
    /// Net area: outer minus holes, mm².
    pub fn area(&self) -> f64 {
        let mut it = self.contours.iter();
        let outer = it.next().map_or(0.0, |c| c.signed_area().abs());
        outer - it.map(|c| c.signed_area().abs()).sum::<f64>()
    }
    pub fn cut_length(&self) -> f64 {
        self.contours.iter().map(Contour::length).sum()
    }
    pub fn pierces(&self) -> usize {
        self.contours.len()
    }
    fn finish(name: &str, mut contours: Vec<Contour>, open_chains: usize) -> Result<Part, String> {
        contours.retain(|c| c.pts.len() >= 3 && c.signed_area().abs() > 1e-6);
        if contours.is_empty() {
            return Err(format!("{name}: no closed outline found"));
        }
        contours.sort_by(|a, b| b.signed_area().abs().total_cmp(&a.signed_area().abs()));
        let (lo, hi) = contours.first().map(Contour::bounds).unwrap_or(([0.0; 2], [0.0; 2]));
        // Keep only loops inside the outer box: separate parts in one file are not holes.
        contours.retain(|c| {
            let (a, b) = c.bounds();
            a[0] >= lo[0] - JOIN_TOL && a[1] >= lo[1] - JOIN_TOL && b[0] <= hi[0] + JOIN_TOL && b[1] <= hi[1] + JOIN_TOL
        });
        let shift = move |p: P2| [p[0] - lo[0], p[1] - lo[1]];
        let mut contours: Vec<Contour> = contours.iter().map(|c| c.map(&shift)).collect();
        // Plasma direction: outer clockwise, holes counter-clockwise, so the good side is always on the right.
        for (i, c) in contours.iter_mut().enumerate() {
            if (i == 0) == (c.signed_area() > 0.0) {
                *c = c.reversed();
            }
        }
        Ok(Part {
            name: name.to_string(),
            contours,
            width: hi[0] - lo[0],
            height: hi[1] - lo[1],
            qty: 1,
            open_chains,
            side: Side::Outside,
            reverse: false,
            flip_lead: false,
        })
    }
}

/// An open run of segments from a drawing.
struct Piece {
    start: P2,
    segs: Vec<Seg>,
}

impl Piece {
    fn end(&self) -> P2 {
        self.segs.last().map_or(self.start, Seg::end)
    }
    fn reverse(self) -> Piece {
        let mut pts = vec![self.start];
        pts.extend(self.segs.iter().map(Seg::end));
        let end = pts.last().copied().unwrap_or(self.start);
        let segs = self
            .segs
            .iter()
            .enumerate()
            .rev()
            .map(|(i, s)| {
                let to = pts.get(i).copied().unwrap_or(self.start);
                match *s {
                    Seg::Line(_) => Seg::Line(to),
                    Seg::Arc { c, ccw, .. } => Seg::Arc { to, c, ccw: !ccw },
                }
            })
            .collect();
        Piece { start: end, segs }
    }
}

fn v(p: &solidcraft_engine::geom::Vec2) -> P2 {
    [p.x, p.y]
}

/// Paths from drawing entities: closed loops come back closed, the rest as open pieces.
fn pieces(geoms: &[Geom2]) -> (Vec<Contour>, Vec<Piece>) {
    let mut closed = Vec::new();
    let mut open = Vec::new();
    for g in geoms {
        match g {
            Geom2::Line(a, b) if dist(v(a), v(b)) > 1e-9 => open.push(Piece { start: v(a), segs: vec![Seg::Line(v(b))] }),
            Geom2::Circle(c, r) if *r > 1e-6 && r.is_finite() => closed.push(Contour::circle(v(c), *r)),
            Geom2::Arc { c, a, b } => {
                if dist(v(a), v(c)) < 1e-6 {
                    continue;
                }
                if dist(v(a), v(b)) < 1e-9 {
                    closed.push(Contour::circle(v(c), dist(v(a), v(c))));
                } else {
                    open.push(Piece { start: v(a), segs: vec![Seg::Arc { to: v(b), c: v(c), ccw: true }] });
                }
            }
            Geom2::Ellipse { c, major, minor } => {
                let ma = (major.x.powi(2) + major.y.powi(2)).sqrt();
                if !(ma > 0.0) || !(*minor > 0.0) {
                    continue;
                }
                let ang = major.y.atan2(major.x);
                let p = (0..120)
                    .map(|i| {
                        let t = TAU * i as f64 / 120.0;
                        let (x, y) = (ma * t.cos(), minor * t.sin());
                        [c.x + x * ang.cos() - y * ang.sin(), c.y + x * ang.sin() + y * ang.cos()]
                    })
                    .collect();
                closed.push(Contour::from_points(p));
            }
            Geom2::Spline { pts, .. } if pts.len() >= 2 => {
                let mut it = pts.iter().map(v);
                if let Some(start) = it.next() {
                    open.push(Piece { start, segs: it.map(Seg::Line).collect() });
                }
            }
            Geom2::Conic { a, apex, b, .. } => {
                let segs = (1..=16)
                    .map(|i| {
                        let t = i as f64 / 16.0;
                        let u = 1.0 - t;
                        Seg::Line([u * u * a.x + 2.0 * u * t * apex.x + t * t * b.x, u * u * a.y + 2.0 * u * t * apex.y + t * t * b.y])
                    })
                    .collect();
                open.push(Piece { start: v(a), segs });
            }
            _ => {}
        }
    }
    (closed, open)
}

/// Join open pieces end to end into loops. Returns closed loops and the count left open.
fn chain(mut open: Vec<Piece>) -> (Vec<Contour>, usize) {
    let mut loops = Vec::new();
    let mut left_open = 0;
    while let Some(mut cur) = open.pop() {
        loop {
            let end = cur.end();
            if !cur.segs.is_empty() && dist(cur.start, end) < JOIN_TOL && (cur.segs.len() > 1 || matches!(cur.segs.first(), Some(Seg::Arc { .. }))) {
                // Snap the closing point exactly.
                if let Some(last) = cur.segs.last_mut() {
                    *last = match *last {
                        Seg::Line(_) => Seg::Line(cur.start),
                        Seg::Arc { c, ccw, .. } => Seg::Arc { to: cur.start, c, ccw },
                    };
                }
                loops.push(Contour::new(cur.start, cur.segs));
                break;
            }
            let found = open.iter().position(|p| dist(p.start, end) < JOIN_TOL || dist(p.end(), end) < JOIN_TOL);
            let Some(i) = found else {
                left_open += 1;
                break;
            };
            let mut next = open.swap_remove(i);
            if dist(next.start, end) >= JOIN_TOL {
                next = next.reverse();
            }
            cur.segs.extend(next.segs);
        }
    }
    (loops, left_open)
}

/// A part from a drawing's entities (mm). The largest loop is the outer profile; loops inside it are holes.
pub fn part_from_geoms(name: &str, geoms: &[Geom2]) -> Result<Part, String> {
    let (mut loops, open) = pieces(geoms);
    let (chained, open_chains) = chain(open);
    loops.extend(chained);
    Part::finish(name, loops, open_chains)
}

/// A part from closed contours (largest is the outline).
#[cfg(test)]
pub fn part_from_contours(name: &str, contours: Vec<Contour>) -> Result<Part, String> {
    Part::finish(name, contours, 0)
}

/// A part from a DXF file's text.
pub fn part_from_dxf(name: &str, text: &str) -> Result<Part, String> {
    let geoms = solidcraft_engine::io::read_dxf(text).map_err(|e| format!("{name}: {e}"))?;
    part_from_geoms(name, &geoms)
}

/// Ear-clipping triangles of a simple polygon, as index triples (for shading parts on screen).
pub fn triangulate(pts: &[P2]) -> Vec<[usize; 3]> {
    let n = pts.len();
    if !(3..=20_000).contains(&n) {
        return Vec::new();
    }
    let area: f64 = (0..n).filter_map(|i| Some((*pts.get(i)?, *pts.get((i + 1) % n)?))).map(|(p, q)| p[0] * q[1] - q[0] * p[1]).sum();
    let mut idx: Vec<usize> = (0..n).collect();
    if area < 0.0 {
        idx.reverse();
    }
    let at = |i: usize| pts.get(i).copied().unwrap_or([0.0, 0.0]);
    let cross = |a: P2, b: P2, c: P2| (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]);
    let mut tris = Vec::with_capacity(n);
    let mut guard = 0usize;
    while idx.len() > 3 && guard < n * 4 {
        guard += 1;
        let m = idx.len();
        let mut clipped = false;
        for k in 0..m {
            let (Some(&ia), Some(&ib), Some(&ic)) = (idx.get((k + m - 1) % m), idx.get(k), idx.get((k + 1) % m)) else { continue };
            let (a, b, c) = (at(ia), at(ib), at(ic));
            if cross(a, b, c) <= 1e-12 {
                continue;
            }
            let inside = idx.iter().any(|&j| {
                if j == ia || j == ib || j == ic {
                    return false;
                }
                let p = at(j);
                cross(a, b, p) >= 0.0 && cross(b, c, p) >= 0.0 && cross(c, a, p) >= 0.0
            });
            if !inside {
                tris.push([ia, ib, ic]);
                idx.remove(k);
                clipped = true;
                break;
            }
        }
        if !clipped {
            break;
        }
    }
    if let [a, b, c] = idx[..] {
        tris.push([a, b, c]);
    }
    tris
}

// --- Sample parts (for trying the workspace without files) -------------------------------------

fn rect(x: f64, y: f64, w: f64, h: f64) -> Contour {
    Contour::from_points(vec![[x, y], [x + w, y], [x + w, y + h], [x, y + h]])
}

pub fn sample_parts() -> Vec<Part> {
    let i = MM_PER_IN;
    let make = |name: &str, loops: Vec<Contour>, qty: u32| {
        let mut p = Part::finish(name, loops, 0).unwrap_or_else(|_| Part {
            name: name.into(),
            contours: Vec::new(),
            width: 0.0,
            height: 0.0,
            qty: 0,
            open_chains: 0,
            side: Side::Outside,
            reverse: false,
            flip_lead: false,
        });
        p.qty = qty;
        p
    };
    // Base plate 8" x 8" with rounded corners and four 9/16" holes.
    let r = 0.75 * i;
    let w = 8.0 * i;
    let plate_outline = Contour::new(
        [r, 0.0],
        vec![
            Seg::Line([w - r, 0.0]),
            Seg::Arc { to: [w, r], c: [w - r, r], ccw: true },
            Seg::Line([w, w - r]),
            Seg::Arc { to: [w - r, w], c: [w - r, w - r], ccw: true },
            Seg::Line([r, w]),
            Seg::Arc { to: [0.0, w - r], c: [r, w - r], ccw: true },
            Seg::Line([0.0, r]),
            Seg::Arc { to: [r, 0.0], c: [r, r], ccw: true },
        ],
    );
    let mut plate = vec![plate_outline];
    for (x, y) in [(1.0, 1.0), (7.0, 1.0), (1.0, 7.0), (7.0, 7.0)] {
        plate.push(Contour::circle([x * i, y * i], 0.28125 * i));
    }
    plate.push(Contour::circle([4.0 * i, 4.0 * i], 2.0 * i));
    // Gusset: right triangle with a clipped corner.
    let gusset = vec![Contour::from_points(vec![[0.0, 0.0], [6.0 * i, 0.0], [6.0 * i, 0.75 * i], [0.75 * i, 6.0 * i], [0.0, 6.0 * i]])];
    // Flange ring 6" OD, 3" ID, 6 bolt holes.
    let mut flange = vec![Contour::circle([3.0 * i, 3.0 * i], 3.0 * i), Contour::circle([3.0 * i, 3.0 * i], 1.5 * i)];
    for k in 0..6 {
        let a = TAU * k as f64 / 6.0;
        flange.push(Contour::circle([3.0 * i + 2.25 * i * a.cos(), 3.0 * i + 2.25 * i * a.sin()], 0.21875 * i));
    }
    // L bracket with a round-ended slot.
    let bracket = vec![
        Contour::from_points(vec![[0.0, 0.0], [10.0 * i, 0.0], [10.0 * i, 3.0 * i], [3.0 * i, 3.0 * i], [3.0 * i, 7.0 * i], [0.0, 7.0 * i]]),
        Contour::new(
            [5.0 * i, 1.25 * i],
            vec![
                Seg::Line([8.0 * i, 1.25 * i]),
                Seg::Arc { to: [8.0 * i, 1.75 * i], c: [8.0 * i, 1.5 * i], ccw: true },
                Seg::Line([5.0 * i, 1.75 * i]),
                Seg::Arc { to: [5.0 * i, 1.25 * i], c: [5.0 * i, 1.5 * i], ccw: true },
            ],
        ),
        Contour::circle([1.5 * i, 5.5 * i], 0.375 * i),
    ];
    // Tab 3" x 2" with a hole.
    let tab = vec![rect(0.0, 0.0, 3.0 * i, 2.0 * i), Contour::circle([1.5 * i, 1.0 * i], 0.3125 * i)];
    // A small washer that can nest inside the plate's centre hole.
    let washer = vec![Contour::circle([1.25 * i, 1.25 * i], 1.25 * i), Contour::circle([1.25 * i, 1.25 * i], 0.5 * i)];
    vec![
        make("base_plate.dxf", plate, 4),
        make("gusset.dxf", gusset, 8),
        make("flange_6in.dxf", flange, 3),
        make("l_bracket.dxf", bracket, 4),
        make("mount_tab.dxf", tab, 12),
        make("washer_2.5in.dxf", washer, 6),
    ]
}

// --- True-shape nesting ------------------------------------------------------------------------

/// Where one part instance sits: placed point = rotate(angle) · p + (tx, ty).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Placement {
    pub part: usize,
    pub sheet: usize,
    pub angle_deg: f64,
    pub tx: f64,
    pub ty: f64,
}

impl Placement {
    pub fn apply(&self, p: P2) -> P2 {
        let (s, c) = self.angle_deg.to_radians().sin_cos();
        [c * p[0] - s * p[1] + self.tx, s * p[0] + c * p[1] + self.ty]
    }
    pub fn contour(&self, c: &Contour) -> Contour {
        c.map(&|p| self.apply(p))
    }
}

#[derive(Clone, Debug, Default)]
pub struct Nest {
    pub placements: Vec<Placement>,
    pub sheets: usize,
    /// Instances that fit on no sheet (larger than the usable sheet).
    pub unplaced: Vec<usize>,
    /// Instances left out because the job was over `MAX_INSTANCES`.
    pub truncated: usize,
    /// Grid cell the nest used, mm.
    pub cell: f64,
}

#[derive(Clone, Copy, Debug)]
pub struct NestSettings {
    pub sheet_w: f64,
    pub sheet_h: f64,
    pub spacing: f64,
    pub margin: f64,
    /// Rotation step in degrees; 0 = no rotation.
    pub rotation_step: f64,
    pub max_sheets: usize,
    /// Fill parts into the holes of larger parts.
    pub part_in_part: bool,
}

/// `rotation_step` value for "Any (best fit)": nest several ways (hull-edge angles, quarter turns, half turns)
/// and keep the layout that uses the fewest sheets and the least sheet length.
pub const BEST_FIT: f64 = -1.0;
/// `rotation_step` value for one of best fit's tries: each part's quarter turns plus the angles that lay its long
/// hull edges square to the sheet.
pub const HULL_ANGLES: f64 = -2.0;

/// Convex hull of points (counter-clockwise, monotone chain).
fn hull(pts: &[P2]) -> Vec<P2> {
    let mut p: Vec<P2> = pts.iter().copied().filter(|q| q[0].is_finite() && q[1].is_finite()).collect();
    p.sort_by(|a, b| a[0].total_cmp(&b[0]).then(a[1].total_cmp(&b[1])));
    p.dedup();
    if p.len() < 3 {
        return p;
    }
    let cross = |o: P2, a: P2, b: P2| (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
    let mut h: Vec<P2> = Vec::with_capacity(p.len() * 2);
    for pass in 0..2 {
        let start = h.len();
        let iter: Box<dyn Iterator<Item = &P2>> = if pass == 0 { Box::new(p.iter()) } else { Box::new(p.iter().rev()) };
        for q in iter {
            while h.len() >= start + 2 && h.get(h.len() - 2).zip(h.last()).is_some_and(|(a, b)| cross(*a, *b, *q) <= 0.0) {
                h.pop();
            }
            h.push(*q);
        }
        h.pop();
    }
    h
}

/// Best-fit angles for a part: the four quarter turns, plus quarter turns that lay each of its longest hull edges
/// along the sheet's X axis (most parts pack tightest with a long straight edge square to the sheet or a neighbour).
pub fn best_fit_angles(part: &Part) -> Vec<f64> {
    let h = hull(part.contours.first().map_or(&[][..], |c| &c.pts[..]));
    let n = h.len();
    let mut edges: Vec<(f64, f64)> = (0..n)
        .filter_map(|i| {
            let (a, b) = (h.get(i)?, h.get((i + 1) % n)?);
            Some((dist(*a, *b), (b[1] - a[1]).atan2(b[0] - a[0]).to_degrees()))
        })
        .collect();
    edges.sort_by(|a, b| b.0.total_cmp(&a.0));
    let mut out: Vec<f64> = vec![0.0, 90.0, 180.0, 270.0];
    for (_, ang) in edges.iter().take(4) {
        for k in 0..4 {
            let a = (-ang + 90.0 * k as f64).rem_euclid(360.0);
            if !out.iter().any(|o| {
                let d = (o - a).rem_euclid(360.0);
                d.min(360.0 - d) < 1.0
            }) {
                out.push(a);
            }
        }
    }
    out.truncate(20);
    out
}

impl NestSettings {
    fn angles_for(&self, part: &Part) -> Vec<f64> {
        if self.rotation_step == BEST_FIT || self.rotation_step == HULL_ANGLES {
            return best_fit_angles(part);
        }
        self.angles()
    }
    fn angles(&self) -> Vec<f64> {
        if !(self.rotation_step > 0.5) {
            return vec![0.0];
        }
        let n = ((360.0 / self.rotation_step).round() as usize).clamp(1, 72);
        (0..n).map(|k| k as f64 * 360.0 / n as f64).collect()
    }
}

/// A packed bitmap: `w` columns × `h` rows.
#[derive(Clone)]
struct Bits {
    w: usize,
    h: usize,
    words: usize,
    data: Vec<u64>,
}

impl Bits {
    fn new(w: usize, h: usize) -> Bits {
        let words = w.div_ceil(64).max(1);
        Bits { w, h, words, data: vec![0; words * h] }
    }
    fn row(&self, y: usize) -> &[u64] {
        self.data.get(y * self.words..(y + 1) * self.words).unwrap_or(&[])
    }
    fn set(&mut self, x: usize, y: usize) {
        if x < self.w
            && y < self.h
            && let Some(word) = self.data.get_mut(y * self.words + x / 64)
        {
            *word |= 1u64 << (x % 64);
        }
    }
    /// Does `mask`, placed with its (0,0) at column `x`, row `y`, hit anything set? The mask's rows are taken
    /// in a spread-out order and only their set words: a part that overlaps something usually shows it within
    /// its first few rows checked.
    fn hits_fast(&self, mask: &Bits, rows: &[(usize, usize, usize)], x: i64, y: i64) -> bool {
        for &(r, k0, k1) in rows {
            let sy = y + r as i64;
            if sy < 0 || sy >= self.h as i64 {
                continue;
            }
            let mrow = mask.row(r);
            for k in k0..k1 {
                let m = mrow.get(k).copied().unwrap_or(0);
                if m != 0 && self.get64(sy as usize, x + 64 * k as i64) & m != 0 {
                    return true;
                }
            }
        }
        false
    }
    /// 64 cells of row `y` starting at column `x` (outside the map reads empty).
    fn get64(&self, y: usize, x: i64) -> u64 {
        let row = self.row(y);
        let word = |i: i64| -> u64 { if i < 0 { 0 } else { row.get(i as usize).copied().unwrap_or(0) } };
        let w = x.div_euclid(64);
        let s = x.rem_euclid(64) as u32;
        if s == 0 { word(w) } else { (word(w) >> s) | (word(w + 1) << (64 - s)) }
    }
    fn or_at(&mut self, mask: &Bits, x: usize, y: usize) {
        for r in 0..mask.h {
            for c in 0..mask.w {
                let on = mask.row(r).get(c / 64).is_some_and(|w| w >> (c % 64) & 1 == 1);
                if on {
                    self.set(x + c, y + r);
                }
            }
        }
    }
}

/// A part at one rotation, rasterised: `body` is every cell the part touches (holes stay empty), `halo`
/// is the body grown by the spacing, offset by `grow` cells.
struct Shape {
    angle: f64,
    /// Rotated part's box minimum (to place its corner on a cell).
    lo: P2,
    body: Bits,
    halo: Bits,
    grow: usize,
    /// The halo's non-empty rows, spread out (row, first word, end word), for `Bits::hits_fast`.
    rows: Vec<(usize, usize, usize)>,
}

impl Shape {
    fn blocked(&self, grid: &Bits, x: usize, y: usize) -> bool {
        grid.hits_fast(&self.halo, &self.rows, x as i64 - self.grow as i64, y as i64 - self.grow as i64)
    }
}

/// 0..n in a spread-out order (bit-reversed), so neighbouring checks probe far-apart rows.
fn spread(n: usize) -> Vec<usize> {
    if n == 0 {
        return Vec::new();
    }
    let bits = usize::BITS - n.leading_zeros();
    (0..(1usize << bits)).map(|i| i.reverse_bits() >> (usize::BITS - bits)).filter(|&i| i < n).collect()
}

fn raster(part: &Part, angle: f64, cell: f64, spacing: f64, holes_free: bool, limit: (usize, usize)) -> Option<Shape> {
    let (s, c) = angle.to_radians().sin_cos();
    let rot = |p: P2| [c * p[0] - s * p[1], s * p[0] + c * p[1]];
    let loops: Vec<Vec<P2>> = part.contours.iter().map(|k| k.pts.iter().map(|p| rot(*p)).collect()).collect();
    let mut lo = [f64::INFINITY; 2];
    let mut hi = [f64::NEG_INFINITY; 2];
    for p in loops.first().into_iter().flatten() {
        lo = [lo[0].min(p[0]), lo[1].min(p[1])];
        hi = [hi[0].max(p[0]), hi[1].max(p[1])];
    }
    if !lo[0].is_finite() {
        lo = [0.0; 2];
        hi = [0.0; 2];
    }
    let w = ((hi[0] - lo[0]) / cell).ceil() as usize + 1;
    let h = ((hi[1] - lo[1]) / cell).ceil() as usize + 1;
    // A part larger than the sheet never fits: don't allocate its grid.
    if w > limit.0 || h > limit.1 || !(hi[0] - lo[0]).is_finite() || !(hi[1] - lo[1]).is_finite() {
        return None;
    }
    let mut grid = vec![false; w * h];
    let mut mark = |x: f64, y: f64| {
        let cx = ((x - lo[0]) / cell).floor();
        let cy = ((y - lo[1]) / cell).floor();
        if cx >= 0.0
            && cy >= 0.0
            && (cx as usize) < w
            && (cy as usize) < h
            && let Some(g) = grid.get_mut(cy as usize * w + cx as usize)
        {
            *g = true;
        }
    };
    // Edges: sample finely so every touched cell is marked.
    let filled: Vec<&Vec<P2>> = if holes_free { loops.iter().collect() } else { loops.iter().take(1).collect() };
    for l in &filled {
        let n = l.len();
        for i in 0..n {
            let (Some(a), Some(b)) = (l.get(i), l.get((i + 1) % n)) else { continue };
            let steps = ((dist(*a, *b) / (cell * 0.25)).ceil() as usize).clamp(1, 1_000_000);
            for k in 0..=steps {
                let p = lerp(*a, *b, k as f64 / steps as f64);
                mark(p[0], p[1]);
            }
        }
    }
    // Interior: even-odd fill at each row's centre line (holes stay empty).
    for y in 0..h {
        let yc = lo[1] + (y as f64 + 0.5) * cell;
        let mut xs: Vec<f64> = Vec::new();
        for l in &filled {
            let n = l.len();
            for i in 0..n {
                let (Some(a), Some(b)) = (l.get(i), l.get((i + 1) % n)) else { continue };
                if (a[1] <= yc) != (b[1] <= yc) {
                    xs.push(a[0] + (yc - a[1]) / (b[1] - a[1]) * (b[0] - a[0]));
                }
            }
        }
        xs.sort_by(f64::total_cmp);
        for pair in xs.chunks(2) {
            if let [x0, x1] = *pair {
                let c0 = ((x0 - lo[0]) / cell - 0.5).ceil().max(0.0) as usize;
                let c1 = ((x1 - lo[0]) / cell - 0.5).floor();
                if c1 < 0.0 {
                    continue;
                }
                for x in c0..=(c1 as usize).min(w - 1) {
                    if let Some(g) = grid.get_mut(y * w + x) {
                        *g = true;
                    }
                }
            }
        }
    }
    let mut body = Bits::new(w, h);
    for y in 0..h {
        for x in 0..w {
            if grid.get(y * w + x).copied().unwrap_or(false) {
                body.set(x, y);
            }
        }
    }
    // Halo: grow by the spacing (square neighbourhood), plus a cell for sampling slack.
    let grow = (((spacing.max(0.0) + 0.3 * cell) / cell).ceil() as usize).min(2000);
    let (hw, hh) = (w + 2 * grow, h + 2 * grow);
    let mut horiz = vec![false; hw * h];
    for y in 0..h {
        let mut last: Option<usize> = None;
        for x in 0..w {
            if grid.get(y * w + x).copied().unwrap_or(false) {
                let from = last.map_or(x, |l| (l + 1).max(x));
                for xx in from..=x + 2 * grow {
                    if let Some(g) = horiz.get_mut(y * hw + xx) {
                        *g = true;
                    }
                }
                last = Some(x + 2 * grow);
            }
        }
    }
    let mut halo = Bits::new(hw, hh);
    for x in 0..hw {
        let mut last: Option<usize> = None;
        for y in 0..h {
            if horiz.get(y * hw + x).copied().unwrap_or(false) {
                let from = last.map_or(y, |l| (l + 1).max(y));
                for yy in from..=y + 2 * grow {
                    halo.set(x, yy);
                }
                last = Some(y + 2 * grow);
            }
        }
    }
    let rows = spread(halo.h)
        .into_iter()
        .filter_map(|r| {
            let row = halo.row(r);
            let k0 = row.iter().position(|w| *w != 0)?;
            let k1 = row.iter().rposition(|w| *w != 0)? + 1;
            Some((r, k0, k1))
        })
        .collect();
    Some(Shape { angle, lo, body, halo, grow, rows })
}

/// True-shape nesting on a grid: each part, biggest first, goes where its right edge is nearest the origin (then
/// lowest) at any allowed rotation, so the offcut stays one clean strip across the far end of the sheet, without its spacing halo touching anything placed. Holes in placed parts stay free, so
/// small parts fill them.
pub fn nest(parts: &[Part], s: &NestSettings) -> Nest {
    if s.rotation_step != BEST_FIT {
        return nest_once(parts, s);
    }
    let mut best: Option<((usize, usize, i64), Nest)> = None;
    let quick = [HULL_ANGLES, 90.0, 180.0].map(|step| nest_once(parts, &NestSettings { rotation_step: step, ..*s }));
    for n in quick.into_iter().chain(std::iter::once(nest_search(parts, s, SEARCH_BUDGET))) {
        let sc = score(parts, &n);
        if best.as_ref().is_none_or(|(b, _)| sc < *b) {
            best = Some((sc, n));
        }
    }
    best.map(|(_, n)| n).unwrap_or_default()
}

/// How a nest run picks among the places and turns that fit.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Rule {
    /// Smallest right edge, then lowest: fills along the sheet's width first, offcut at the far end.
    RightEdge,
    /// Lowest top edge, then leftmost.
    TopEdge,
    /// Keep everything placed so far in the smallest square, then the smallest box.
    Compact,
}

/// One rotated part image per angle for every part, and the grid they were drawn on.
struct Prepared {
    cell: f64,
    gw: usize,
    gh: usize,
    shapes: Vec<Vec<Shape>>,
}

fn prepare(parts: &[Part], s: &NestSettings, angles: &dyn Fn(&Part) -> Vec<f64>) -> Option<Prepared> {
    let uw = s.sheet_w - 2.0 * s.margin;
    let uh = s.sheet_h - 2.0 * s.margin;
    if !(uw > 1.0) || !(uh > 1.0) || !uw.is_finite() || !uh.is_finite() {
        return None;
    }
    // About a million cells per sheet, never finer than 0.5 mm.
    let cell = ((uw * uh / 1.0e6).sqrt()).clamp(0.5, 25.0);
    let gw = (uw / cell).floor() as usize;
    let gh = (uh / cell).floor() as usize;
    let shapes = parts
        .iter()
        .map(|p| {
            if p.qty == 0 { Vec::new() } else { angles(p).iter().filter_map(|a| raster(p, *a, cell, s.spacing, s.part_in_part, (gw, gh))).collect() }
        })
        .collect();
    Some(Prepared { cell, gw, gh, shapes })
}

fn instances(parts: &[Part], out: &mut Nest) -> Vec<usize> {
    let mut items: Vec<usize> = Vec::new();
    for (i, p) in parts.iter().enumerate() {
        for _ in 0..p.qty {
            if items.len() >= MAX_INSTANCES {
                out.truncated += 1;
            } else {
                items.push(i);
            }
        }
    }
    items.sort_by(|a, b| {
        let area = |i: usize| parts.get(i).map_or(0.0, |p| p.width * p.height);
        area(*b).total_cmp(&area(*a)).then(a.cmp(b))
    });
    items
}

/// Multi-start: every rule, and every turn of the first part, keeping the best layout. (Experiment.)
/// Turns tried by the "Any (best fit)" search: every 15° plus the part's long hull edges laid flat.
fn search_angles(part: &Part) -> Vec<f64> {
    let mut out: Vec<f64> = (0..24).map(|k| k as f64 * 15.0).collect();
    for a in best_fit_angles(part) {
        if !out.iter().any(|o| {
            let d = (o - a).rem_euclid(360.0);
            d.min(360.0 - d) < 1.0
        }) {
            out.push(a);
        }
    }
    out
}

/// Score of a finished nest: fewer unplaced parts, then fewer sheets, then less sheet length used.
fn score(parts: &[Part], n: &Nest) -> (usize, usize, i64) {
    let length: f64 = (0..n.sheets).map(|k| used_length(parts, n, k)).sum();
    (n.unplaced.len() + n.truncated, n.sheets, (length * 100.0).round() as i64)
}

/// How long the "Any (best fit)" search may run before it settles for the best nest found so far.
#[cfg(not(target_arch = "wasm32"))]
pub const SEARCH_BUDGET: web_time::Duration = web_time::Duration::from_secs(8);
/// On the web the search shares the window's only thread, so it gets a short budget.
#[cfg(target_arch = "wasm32")]
pub const SEARCH_BUDGET: web_time::Duration = web_time::Duration::from_millis(1500);

/// Multi-start search: every placement rule × every turn of the first (largest) part, run across the
/// machine's cores until `budget` runs out or a nest needs no more sheets than the area allows.
pub fn nest_search(parts: &[Part], s: &NestSettings, budget: web_time::Duration) -> Nest {
    use std::sync::Mutex;
    use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
    let mut base = Nest::default();
    let items = instances(parts, &mut base);
    let Some(prep) = prepare(parts, s, &search_angles) else {
        base.unplaced = items;
        return base;
    };
    let firsts = items.first().and_then(|i| prep.shapes.get(*i)).map_or(0, Vec::len).max(1);
    let rules = [Rule::RightEdge, Rule::TopEdge, Rule::Compact];
    let starts: Vec<(Rule, Option<usize>)> =
        std::iter::once((Rule::RightEdge, None)).chain((0..firsts).flat_map(|f| rules.iter().map(move |r| (*r, Some(f))))).collect();
    // Fewest sheets the parts' area could ever need: reaching it ends the search early.
    let usable = ((s.sheet_w - 2.0 * s.margin.max(0.0)) * (s.sheet_h - 2.0 * s.margin.max(0.0))).max(1e-9);
    let area: f64 = items.iter().filter_map(|i| parts.get(*i)).map(Part::area).sum();
    let floor = ((area / usable).ceil() as usize).max(1);
    let deadline = web_time::Instant::now() + budget;
    let next = AtomicUsize::new(0);
    let done = AtomicBool::new(false);
    let best: Mutex<Option<((usize, usize, i64), Nest)>> = Mutex::new(None);
    let work = || loop {
        if done.load(Ordering::Relaxed) || web_time::Instant::now() >= deadline {
            return;
        }
        let k = next.fetch_add(1, Ordering::Relaxed);
        let Some((rule, first)) = starts.get(k).copied() else {
            return;
        };
        let n = run(parts, s, &prep, &items, rule, first, base.clone());
        let sc = score(parts, &n);
        if sc.0 == 0 && sc.1 <= floor {
            done.store(true, Ordering::Relaxed);
        }
        if let Ok(mut b) = best.lock()
            && b.as_ref().is_none_or(|(old, _)| sc < *old)
        {
            *b = Some((sc, n));
        }
    };
    #[cfg(not(target_arch = "wasm32"))]
    {
        let threads = std::thread::available_parallelism().map_or(1, |n| n.get()).clamp(1, 16);
        std::thread::scope(|scope| {
            for _ in 1..threads {
                scope.spawn(work);
            }
            work();
        });
    }
    #[cfg(target_arch = "wasm32")]
    work();
    best.into_inner().ok().flatten().map(|(_, n)| n).unwrap_or(base)
}

fn nest_once(parts: &[Part], s: &NestSettings) -> Nest {
    let mut out = Nest::default();
    let items = instances(parts, &mut out);
    let Some(prep) = prepare(parts, s, &|p| s.angles_for(p)) else {
        out.unplaced = items;
        return out;
    };
    run(parts, s, &prep, &items, Rule::RightEdge, None, out)
}

/// The first place a part image fits, scanning columns from the origin (lowest in the first column with room)
/// or rows (leftmost in the first row with room). `bound` stops the scan once nothing can beat it.
fn first_fit(grid: &Bits, sh: &Shape, gw: usize, gh: usize, by_rows: bool, bound: Option<usize>) -> Option<(usize, usize)> {
    let (w, h) = (sh.body.w, sh.body.h);
    if w > gw || h > gh {
        return None;
    }
    let free = |x: usize, y: usize| !sh.blocked(grid, x, y);
    if by_rows {
        for y in 0..=(gh - h) {
            if bound.is_some_and(|b| y + h >= b) {
                return None;
            }
            for x in 0..=(gw - w) {
                if free(x, y) {
                    return Some((x, y));
                }
            }
        }
    } else {
        for x in 0..=(gw - w) {
            if bound.is_some_and(|b| x + w >= b) {
                return None;
            }
            for y in 0..=(gh - h) {
                if free(x, y) {
                    return Some((x, y));
                }
            }
        }
    }
    None
}

/// One nest run: each part in turn, on the first sheet that takes it, at the place and turn `rule` likes best.
/// `first_turn` forces the image of the first part on each sheet (multi-start search).
fn run(parts: &[Part], s: &NestSettings, prep: &Prepared, items: &[usize], rule: Rule, first_turn: Option<usize>, mut out: Nest) -> Nest {
    let (cell, gw, gh) = (prep.cell, prep.gw, prep.gh);
    out.cell = cell;
    let mut sheets: Vec<Bits> = Vec::new();
    // What each sheet's placed parts reach, in cells (for Compact).
    let mut reach: Vec<(usize, usize)> = Vec::new();
    // Holes of placed parts, per sheet, as cell boxes (x0, y0, x1, y1), for part-in-part.
    let mut holes: Vec<Vec<(usize, usize, usize, usize)>> = Vec::new();
    for &item in items {
        let Some(variants) = prep.shapes.get(item) else { continue };
        let mut placed = false;
        for sheet in 0..=sheets.len() {
            if sheet == sheets.len() {
                if sheets.len() >= s.max_sheets.max(1) {
                    break;
                }
                sheets.push(Bits::new(gw, gh));
                reach.push((0, 0));
            }
            let Some(grid) = sheets.get(sheet) else { break };
            let (rx, ry) = reach.get(sheet).copied().unwrap_or((0, 0));
            // (score, x, y, variant); lower score wins.
            let mut best: Option<((usize, usize, usize), usize, usize, usize)> = None;
            // Part in part: a hole that takes the part wins outright.
            if s.part_in_part {
                'holes: for &(x0, y0, x1, y1) in holes.get(sheet).map(Vec::as_slice).unwrap_or(&[]) {
                    for (vi, sh) in variants.iter().enumerate() {
                        let (w, h) = (sh.body.w, sh.body.h);
                        if x0 + w > x1 || y0 + h > y1 {
                            continue;
                        }
                        for y in y0..=(y1 - h) {
                            for x in x0..=(x1 - w) {
                                if !sh.blocked(grid, x, y) {
                                    best = Some(((0, 0, 0), x, y, vi));
                                    break 'holes;
                                }
                            }
                        }
                    }
                }
            }
            if best.is_none() {
                // The search forces the turn of the first part on each sheet: it decides how the rest interlock.
                let empty = !out.placements.iter().any(|p| p.sheet == sheet);
                let forced = first_turn.filter(|f| empty && *f < variants.len());
                for (vi, sh) in variants.iter().enumerate() {
                    if forced.is_some_and(|f| f != vi) {
                        continue;
                    }
                    let (w, h) = (sh.body.w, sh.body.h);
                    let consider = |x: usize, y: usize, best: &mut Option<((usize, usize, usize), usize, usize, usize)>| {
                        let score = match rule {
                            Rule::RightEdge => (x + w, y, 0),
                            Rule::TopEdge => (y + h, x, 0),
                            Rule::Compact => {
                                let (bx, by) = (rx.max(x + w), ry.max(y + h));
                                (bx.max(by), bx * by, x + y)
                            }
                        };
                        if best.is_none_or(|b| score < b.0) {
                            *best = Some((score, x, y, vi));
                        }
                    };
                    match rule {
                        Rule::RightEdge => {
                            if let Some((x, y)) = first_fit(grid, sh, gw, gh, false, best.map(|b| b.0.0)) {
                                consider(x, y, &mut best);
                            }
                        }
                        Rule::TopEdge => {
                            if let Some((x, y)) = first_fit(grid, sh, gw, gh, true, best.map(|b| b.0.0)) {
                                consider(x, y, &mut best);
                            }
                        }
                        Rule::Compact => {
                            for by_rows in [false, true] {
                                if let Some((x, y)) = first_fit(grid, sh, gw, gh, by_rows, None) {
                                    consider(x, y, &mut best);
                                }
                            }
                        }
                    }
                }
            }
            if let Some((_, x, y, vi)) = best
                && let Some(sh) = variants.get(vi)
            {
                if let Some(grid) = sheets.get_mut(sheet) {
                    grid.or_at(&sh.body, x, y);
                }
                if let Some(r) = reach.get_mut(sheet) {
                    *r = (r.0.max(x + sh.body.w), r.1.max(y + sh.body.h));
                }
                let ox = s.margin + x as f64 * cell;
                let oy = s.margin + y as f64 * cell;
                let pl = Placement { part: item, sheet, angle_deg: sh.angle, tx: ox - sh.lo[0], ty: oy - sh.lo[1] };
                if s.part_in_part
                    && let Some(part) = parts.get(item)
                {
                    while holes.len() <= sheet {
                        holes.push(Vec::new());
                    }
                    if let Some(list) = holes.get_mut(sheet) {
                        for hole in part.contours.iter().skip(1) {
                            let (lo, hi) = pl.contour(hole).bounds();
                            let cx0 = ((lo[0] - s.margin) / cell).floor().max(0.0) as usize;
                            let cy0 = ((lo[1] - s.margin) / cell).floor().max(0.0) as usize;
                            let cx1 = (((hi[0] - s.margin) / cell).ceil().max(0.0) as usize).min(gw);
                            let cy1 = (((hi[1] - s.margin) / cell).ceil().max(0.0) as usize).min(gh);
                            if cx1 > cx0 + 2 && cy1 > cy0 + 2 {
                                list.push((cx0, cy0, cx1, cy1));
                            }
                        }
                    }
                }
                out.placements.push(pl);
                placed = true;
                break;
            }
        }
        if !placed {
            out.unplaced.push(item);
        }
    }
    out.sheets = sheets.len();
    out
}

fn seg_dist(p: P2, a: P2, b: P2) -> f64 {
    let d = [b[0] - a[0], b[1] - a[1]];
    let l2 = d[0] * d[0] + d[1] * d[1];
    if l2 <= 1e-18 {
        return dist(p, a);
    }
    let t = (((p[0] - a[0]) * d[0] + (p[1] - a[1]) * d[1]) / l2).clamp(0.0, 1.0);
    dist(p, lerp(a, b, t))
}

fn segs_cross(a: P2, b: P2, c: P2, d: P2) -> bool {
    let o = |p: P2, q: P2, r: P2| (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0]);
    let (d1, d2, d3, d4) = (o(c, d, a), o(c, d, b), o(a, b, c), o(a, b, d));
    ((d1 > 0.0) != (d2 > 0.0)) && ((d3 > 0.0) != (d4 > 0.0))
}

/// Closest approach of two closed polylines (0 when they cross).
fn loop_gap(a: &[P2], b: &[P2]) -> f64 {
    let (na, nb) = (a.len(), b.len());
    let mut m = f64::INFINITY;
    for i in 0..na {
        let (Some(&p), Some(&q)) = (a.get(i), a.get((i + 1) % na)) else { continue };
        for j in 0..nb {
            let (Some(&r), Some(&t)) = (b.get(j), b.get((j + 1) % nb)) else { continue };
            if segs_cross(p, q, r, t) {
                return 0.0;
            }
            m = m.min(seg_dist(p, r, t)).min(seg_dist(r, p, q));
        }
    }
    m
}

fn point_in(p: P2, pts: &[P2]) -> bool {
    let n = pts.len();
    let mut odd = false;
    for i in 0..n {
        let (Some(a), Some(b)) = (pts.get(i), pts.get((i + 1) % n)) else { continue };
        if (a[1] > p[1]) != (b[1] > p[1]) && p[0] < a[0] + (p[1] - a[1]) / (b[1] - a[1]) * (b[0] - a[0]) {
            odd = !odd;
        }
    }
    odd
}

/// Is `p` on a placed part's solid (inside its outline, not in a hole)?
fn on_solid(p: P2, loops: &[Vec<P2>]) -> bool {
    let mut it = loops.iter();
    it.next().is_some_and(|outer| point_in(p, outer)) && !it.any(|h| point_in(p, h))
}

/// The placement on `sheet` whose solid is under `p` (the last drawn wins), for picking parts by hand.
pub fn placement_at(parts: &[Part], nest: &Nest, sheet: usize, p: P2) -> Option<usize> {
    nest.placements.iter().enumerate().rev().filter(|(_, pl)| pl.sheet == sheet).find_map(|(i, pl)| {
        let part = parts.get(pl.part)?;
        let loops: Vec<Vec<P2>> = part.contours.iter().map(|c| c.pts.iter().map(|q| pl.apply(*q)).collect()).collect();
        on_solid(p, &loops).then_some(i)
    })
}

/// What a placed part is touching after a manual move: the other placements closer than the part spacing (or
/// overlapping), and whether it left the usable sheet.
pub fn contacts(parts: &[Part], nest: &Nest, idx: usize, s: &NestSettings) -> (Vec<usize>, bool) {
    let loops_of = |i: usize| -> Option<Vec<Vec<P2>>> {
        let pl = nest.placements.get(i)?;
        Some(parts.get(pl.part)?.contours.iter().map(|c| c.pts.iter().map(|q| pl.apply(*q)).collect()).collect())
    };
    let Some(me) = loops_of(idx) else { return (Vec::new(), false) };
    let Some(sheet) = nest.placements.get(idx).map(|p| p.sheet) else { return (Vec::new(), false) };
    let bounds = |l: &[Vec<P2>]| {
        let mut lo = [f64::INFINITY; 2];
        let mut hi = [f64::NEG_INFINITY; 2];
        for q in l.first().into_iter().flatten() {
            lo = [lo[0].min(q[0]), lo[1].min(q[1])];
            hi = [hi[0].max(q[0]), hi[1].max(q[1])];
        }
        (lo, hi)
    };
    let (lo, hi) = bounds(&me);
    let m = s.margin.max(0.0);
    let off = lo[0] < m - 0.01 || lo[1] < m - 0.01 || hi[0] > s.sheet_w - m + 0.01 || hi[1] > s.sheet_h - m + 0.01;
    // Contact: closer than the spacing, with a small allowance for the nest grid.
    let gap = (s.spacing.max(0.0) - 0.05).max(0.05);
    let mut hits = Vec::new();
    for (j, pl) in nest.placements.iter().enumerate() {
        if j == idx || pl.sheet != sheet {
            continue;
        }
        let Some(other) = loops_of(j) else { continue };
        let (olo, ohi) = bounds(&other);
        if olo[0] > hi[0] + gap || ohi[0] < lo[0] - gap || olo[1] > hi[1] + gap || ohi[1] < lo[1] - gap {
            continue;
        }
        let near = me.iter().any(|a| other.iter().any(|b| loop_gap(a, b) < gap));
        let inside = me.first().and_then(|l| l.first()).is_some_and(|p| on_solid(*p, &other))
            || other.first().and_then(|l| l.first()).is_some_and(|p| on_solid(*p, &me));
        if near || inside {
            hits.push(j);
        }
    }
    (hits, off)
}

/// Sheet utilisation 0–1: net part area over sheet area, for one sheet.
pub fn utilisation(parts: &[Part], nest: &Nest, sheet: usize, s: &NestSettings) -> f64 {
    let used: f64 = nest.placements.iter().filter(|p| p.sheet == sheet).filter_map(|p| parts.get(p.part)).map(Part::area).sum();
    let total = s.sheet_w * s.sheet_h;
    if total > 0.0 { (used / total).clamp(0.0, 1.0) } else { 0.0 }
}

/// How far along X the nest reaches (the sheet length used), mm.
pub fn used_length(parts: &[Part], nest: &Nest, sheet: usize) -> f64 {
    nest.placements
        .iter()
        .filter(|p| p.sheet == sheet)
        .filter_map(|p| Some(p.contour(parts.get(p.part)?.contours.first()?).bounds().1[0]))
        .fold(0.0, f64::max)
}

// --- Tool library (SheetCAM .tools) ------------------------------------------------------------

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Metal {
    Steel,
    Stainless,
    Aluminum,
    Other,
}

impl Metal {
    pub fn label(self) -> &'static str {
        match self {
            Metal::Steel => "Mild steel",
            Metal::Stainless => "Stainless",
            Metal::Aluminum => "Aluminum",
            Metal::Other => "Other",
        }
    }
    /// Pounds per cubic inch.
    pub fn density(self) -> f64 {
        match self {
            Metal::Steel | Metal::Other => 0.2836,
            Metal::Stainless => 0.289,
            Metal::Aluminum => 0.0975,
        }
    }
}

/// One plasma tool: a SheetCAM tool / QtPlasmaC material.
#[derive(Clone, Debug, PartialEq)]
pub struct Tool {
    pub number: u32,
    pub name: String,
    pub metal: Metal,
    pub oxy: bool,
    pub thickness_in: Option<f64>,
    pub amps: u32,
    pub kerf: f64,
    pub feed: f64,
    pub pierce_delay: f64,
    pub pierce_height: f64,
    pub cut_height: f64,
    pub volts: f64,
    pub puddle_height: f64,
    pub puddle_delay: f64,
    pub pause_at_end: f64,
    /// Consumable wear charged per pierce, dollars.
    pub per_pierce: f64,
}

/// Sheet gauge to inches (manufacturers' standard gauge for steel; aluminium uses the same table here).
fn gauge(g: u32, metal: Metal) -> Option<f64> {
    let steel = [
        (7, 0.1793),
        (8, 0.1644),
        (10, 0.1345),
        (11, 0.1196),
        (12, 0.1046),
        (14, 0.0747),
        (16, 0.0598),
        (18, 0.0478),
        (20, 0.0359),
        (22, 0.0299),
        (24, 0.0239),
        (26, 0.0179),
    ];
    let stainless = [
        (7, 0.1875),
        (8, 0.1719),
        (10, 0.1406),
        (11, 0.125),
        (12, 0.1094),
        (14, 0.0781),
        (16, 0.0625),
        (18, 0.05),
        (20, 0.0375),
        (22, 0.0313),
        (24, 0.025),
        (26, 0.0188),
    ];
    let table: &[(u32, f64)] = if metal == Metal::Stainless { &stainless } else { &steel };
    table.iter().find(|(n, _)| *n == g).map(|(_, t)| *t)
}

fn parse_fraction(s: &str) -> Option<f64> {
    let s = s.trim().trim_end_matches(['"', '\'']);
    let mut total = 0.0;
    let mut any = false;
    for part in s.split_whitespace() {
        if let Some((a, b)) = part.split_once('/') {
            let (a, b) = (a.parse::<f64>().ok()?, b.parse::<f64>().ok()?);
            if b == 0.0 {
                return None;
            }
            total += a / b;
        } else {
            total += part.trim_end_matches("in").parse::<f64>().ok()?;
        }
        any = true;
    }
    any.then_some(total)
}

/// Material, gauge or fraction, and amps from a tool's name ("45A FineCut 12GA Steel", "65A 3/8 Stainless").
fn describe(name: &str) -> (Metal, bool, Option<f64>, u32) {
    let lower = name.to_lowercase();
    let metal = if lower.contains("stainless") {
        Metal::Stainless
    } else if lower.contains("alum") {
        Metal::Aluminum
    } else if lower.contains("steel") || lower.contains("oxy") {
        Metal::Steel
    } else {
        Metal::Other
    };
    let oxy = lower.starts_with("oxy");
    let words: Vec<&str> = name.split_whitespace().collect();
    let amps = words.iter().find_map(|w| w.strip_suffix(['A', 'a']).and_then(|n| n.parse::<u32>().ok())).unwrap_or(0);
    let mut thickness = None;
    for (k, w) in words.iter().enumerate() {
        let wl = w.to_lowercase();
        if let Some(g) = wl.strip_suffix("ga").and_then(|n| n.parse::<u32>().ok()) {
            thickness = gauge(g, metal);
            break;
        }
        if w.contains('/') || wl.ends_with("in") || wl.ends_with('"') {
            // "1 1/2"" spans two words.
            let prev = k.checked_sub(1).and_then(|p| words.get(p)).filter(|p| p.parse::<f64>().is_ok());
            let text = match prev {
                Some(p) => format!("{p} {w}"),
                None => (*w).to_string(),
            };
            if let Some(t) = parse_fraction(&text).filter(|t| *t > 0.0 && *t < 12.0) {
                thickness = Some(t);
                break;
            }
        }
    }
    (metal, oxy, thickness, amps)
}

/// Tools from a SheetCAM `.tools` file (INI, metric values).
pub fn parse_tools(text: &str) -> Vec<Tool> {
    let mut tools = Vec::new();
    let mut cur: Option<Vec<(String, String)>> = None;
    let mut custom = false;
    let flush = |kv: Vec<(String, String)>, tools: &mut Vec<Tool>| {
        let get = |k: &str| kv.iter().find(|(key, _)| key == k).map(|(_, v)| v.as_str());
        let num = |k: &str| get(k).and_then(|v| v.trim().parse::<f64>().ok()).filter(|v| v.is_finite()).unwrap_or(0.0);
        if get("Type").is_some_and(|t| t.trim() != "PlasmaTool") {
            return;
        }
        let name = get("Name").unwrap_or("Tool").trim().to_string();
        let (metal, oxy, thickness_in, amps) = describe(&name);
        tools.push(Tool {
            number: num("Tool number").max(0.0) as u32,
            name,
            metal,
            oxy,
            thickness_in,
            amps,
            kerf: num("Kerf width"),
            feed: num("Feed rate"),
            pierce_delay: num("Pierce delay"),
            pierce_height: num("Pierce height"),
            cut_height: num("Cut height"),
            volts: num("Cut voltage"),
            puddle_height: num("Puddle jump height"),
            puddle_delay: num("Puddle jump delay"),
            pause_at_end: num("Pause at end of cut"),
            per_pierce: 0.02 + 0.0015 * f64::from(amps) * thickness_in.unwrap_or(0.1).max(0.05) * 10.0,
        });
    };
    for raw in text.lines().take(200_000) {
        let line = raw.trim();
        if line.starts_with('[') && line.ends_with(']') {
            let section = &line[1..line.len() - 1];
            if section.ends_with("/Custom") {
                custom = true;
                continue;
            }
            custom = false;
            if let Some(kv) = cur.take() {
                flush(kv, &mut tools);
            }
            if section.starts_with("Tool") {
                cur = Some(Vec::new());
            }
            continue;
        }
        let Some((k, v)) = line.split_once('=') else { continue };
        let key = k.trim().replace("\\ ", " ");
        if let Some(kv) = cur.as_mut() {
            // Custom keys (THC, cut voltage, puddle jump) belong to the same tool.
            if custom || !kv.iter().any(|(existing, _)| *existing == key) {
                kv.push((key, v.trim().to_string()));
            }
        }
    }
    if let Some(kv) = cur.take() {
        flush(kv, &mut tools);
    }
    tools.sort_by_key(|t| t.number);
    tools
}

/// The shop's Hypertherm Powermax 65 tool set, bundled as the starting library.
pub fn default_tools() -> Vec<Tool> {
    parse_tools(include_str!("plasma_cam_pmax65.tools"))
}

/// A QtPlasmaC `<machine>_material.cfg` from the tool library, in the machine's units.
pub fn qtplasmac_materials(tools: &[Tool], inch: bool) -> String {
    use std::fmt::Write as _;
    let k = if inch { 1.0 / MM_PER_IN } else { 1.0 };
    let mut out = String::from("# QtPlasmaC material file written by SolidCraft Plasma CAM\n");
    let _ = writeln!(out, "# units: {}\n", if inch { "inch, inch/min" } else { "mm, mm/min" });
    for t in tools {
        let _ = writeln!(out, "[MATERIAL_NUMBER_{}]", t.number);
        let _ = writeln!(out, "NAME               = {}", t.name.replace(['[', ']', '\n'], " "));
        let _ = writeln!(out, "KERF_WIDTH         = {:.4}", t.kerf * k);
        let _ = writeln!(out, "PIERCE_HEIGHT      = {:.4}", t.pierce_height * k);
        let _ = writeln!(out, "PIERCE_DELAY       = {}", t.pierce_delay);
        let _ = writeln!(out, "PUDDLE_JUMP_HEIGHT = {}", t.puddle_height);
        let _ = writeln!(out, "PUDDLE_JUMP_DELAY  = {}", t.puddle_delay);
        let _ = writeln!(out, "CUT_HEIGHT         = {:.4}", t.cut_height * k);
        let _ = writeln!(out, "CUT_SPEED          = {:.1}", t.feed * k);
        let _ = writeln!(out, "CUT_AMPS           = {}", t.amps);
        let _ = writeln!(out, "CUT_VOLTS          = {}", t.volts);
        let _ = writeln!(out, "PAUSE_AT_END       = {}", t.pause_at_end);
        let _ = writeln!(out, "GAS_PRESSURE       = 0");
        let _ = writeln!(out, "CUT_MODE           = 1\n");
    }
    out
}

// --- Quoting -----------------------------------------------------------------------------------

#[derive(Clone, Debug, PartialEq)]
pub struct Pricing {
    pub steel_per_lb: f64,
    pub stainless_per_lb: f64,
    pub aluminum_per_lb: f64,
    pub machine_per_hour: f64,
    pub setup: f64,
    pub markup_pct: f64,
    /// Charge every sheet used in full, or only the parts' share plus waste allowance.
    pub charge_full_sheets: bool,
    pub scrap_allowance_pct: f64,
    /// Rapid moves and torch height finding, seconds per pierce on top of the pierce delay.
    pub overhead_s_per_pierce: f64,
    /// Charge per pierce, dollars (on top of consumable wear).
    pub per_pierce: f64,
    /// Charge per inch of cut, dollars.
    pub per_inch: f64,
    /// Sales tax on the customer quote, percent.
    pub tax_pct: f64,
}

impl Default for Pricing {
    fn default() -> Self {
        Self {
            steel_per_lb: 0.65,
            stainless_per_lb: 2.75,
            aluminum_per_lb: 2.25,
            machine_per_hour: 120.0,
            setup: 25.0,
            markup_pct: 20.0,
            charge_full_sheets: false,
            scrap_allowance_pct: 15.0,
            overhead_s_per_pierce: 2.0,
            per_pierce: 0.0,
            per_inch: 0.0,
            tax_pct: 0.0,
        }
    }
}

impl Pricing {
    pub fn per_lb(&self, m: Metal) -> f64 {
        match m {
            Metal::Steel | Metal::Other => self.steel_per_lb,
            Metal::Stainless => self.stainless_per_lb,
            Metal::Aluminum => self.aluminum_per_lb,
        }
    }
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct QuoteLine {
    pub name: String,
    pub qty: u32,
    pub each: f64,
    pub total: f64,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct Quote {
    pub cut_in: f64,
    pub pierces: usize,
    pub minutes: f64,
    pub weight_lb: f64,
    pub material: f64,
    pub machine: f64,
    pub consumables: f64,
    pub setup: f64,
    /// Pierce charges (pierces × price per pierce).
    pub pierce_charge: f64,
    /// Cut-length charges (inches × price per inch).
    pub inch_charge: f64,
    pub markup: f64,
    pub total: f64,
    pub lines: Vec<QuoteLine>,
}

/// Machine minutes for one part with a tool.
pub fn part_minutes(p: &Part, t: &Tool, overhead_s: f64) -> f64 {
    let feed = if t.feed > 0.0 { t.feed } else { 1000.0 };
    p.cut_length() / feed + p.pierces() as f64 * (t.pierce_delay + overhead_s) / 60.0
}

/// Price a job: every part times its quantity with one tool. `sheets_area_mm2` (all sheets the nest used)
/// replaces the parts' material when `charge_full_sheets` is on.
pub fn quote(parts: &[Part], t: &Tool, pr: &Pricing, sheets_area_mm2: Option<f64>) -> Quote {
    let mut q = Quote::default();
    let markup = 1.0 + pr.markup_pct.max(0.0) / 100.0;
    let thick = t.thickness_in.unwrap_or(0.0);
    let lb = |area_mm2: f64| area_mm2 / (MM_PER_IN * MM_PER_IN) * thick * t.metal.density();
    let mut part_material = 0.0;
    for p in parts {
        let n = f64::from(p.qty);
        let minutes = part_minutes(p, t, pr.overhead_s_per_pierce);
        let weight = lb(p.area());
        let material = weight * (1.0 + pr.scrap_allowance_pct.max(0.0) / 100.0) * pr.per_lb(t.metal);
        let machine = minutes / 60.0 * pr.machine_per_hour;
        let consumables = p.pierces() as f64 * t.per_pierce;
        let rate_charges = p.pierces() as f64 * pr.per_pierce.max(0.0) + p.cut_length() / MM_PER_IN * pr.per_inch.max(0.0);
        q.pierce_charge += p.pierces() as f64 * pr.per_pierce.max(0.0) * n;
        q.inch_charge += p.cut_length() / MM_PER_IN * pr.per_inch.max(0.0) * n;
        q.cut_in += p.cut_length() / MM_PER_IN * n;
        q.pierces += p.pierces() * p.qty as usize;
        q.minutes += minutes * n;
        q.weight_lb += weight * n;
        part_material += material * n;
        q.machine += machine * n;
        q.consumables += consumables * n;
        let each = (material + machine + consumables + rate_charges) * markup;
        q.lines.push(QuoteLine { name: p.name.clone(), qty: p.qty, each, total: each * n });
    }
    q.material = match (pr.charge_full_sheets, sheets_area_mm2) {
        (true, Some(a)) => lb(a) * pr.per_lb(t.metal),
        _ => part_material,
    };
    q.setup = if parts.iter().any(|p| p.qty > 0) { pr.setup } else { 0.0 };
    let base = q.material + q.machine + q.consumables + q.setup + q.pierce_charge + q.inch_charge;
    q.markup = base * (markup - 1.0);
    q.total = base + q.markup;
    // Spread setup and any full-sheet difference across the lines so they add up to the total.
    let lines_sum: f64 = q.lines.iter().map(|l| l.total).sum();
    if lines_sum > 0.0 {
        let k = q.total / lines_sum;
        for l in &mut q.lines {
            l.each *= k;
            l.total *= k;
        }
    }
    q
}

// --- Toolpaths and G-code ----------------------------------------------------------------------

/// Which side of the drawn line the kerf goes (SheetCAM's contour methods).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Side {
    /// The part is kept: outsides cut outside the line, holes inside it.
    #[default]
    Outside,
    /// The drop is kept: outsides cut inside the line, holes outside it.
    Inside,
    /// The torch runs on the line (no kerf offset).
    OnLine,
}

impl Side {
    pub fn label(self) -> &'static str {
        match self {
            Side::Outside => "Outside",
            Side::Inside => "Inside",
            Side::OnLine => "No offset",
        }
    }
}

/// Lead-in or lead-out shape.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Lead {
    None,
    Line,
    Arc,
}

impl Lead {
    pub const ALL: [Lead; 3] = [Lead::None, Lead::Line, Lead::Arc];
    pub fn label(self) -> &'static str {
        match self {
            Lead::None => "None",
            Lead::Line => "Line",
            Lead::Arc => "Arc",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PostSettings {
    /// Program in inches (G20) or millimetres (G21): match the QtPlasmaC machine config.
    pub inch: bool,
    pub lead_in_kind: Lead,
    /// Lead-in length (line) or radius (arc), mm.
    pub lead_in: f64,
    pub lead_out_kind: Lead,
    pub lead_out: f64,
    /// Cut past the start point before leading out, mm.
    pub overcut: f64,
    /// Select the material in QtPlasmaC with M190 and take its feed rate; otherwise write the tool's feed.
    pub use_material: bool,
    /// Kerf compensation by the controller (G41.1 / G42.1 with the material's kerf). Off (the default, as
    /// SheetCAM posts it): the CAM offsets the torch path by half the tool's kerf and the code has no G41.
    pub kerf_comp: bool,
    /// Holes smaller than this diameter run at `hole_speed_pct` (M67 E3); 0 = off.
    pub small_hole: f64,
    pub hole_speed_pct: f64,
    /// Rapid traverse for run-time estimates and the simulation, mm/min.
    pub rapid: f64,
}

impl Default for PostSettings {
    fn default() -> Self {
        Self {
            inch: true,
            lead_in_kind: Lead::Arc,
            lead_in: 0.2 * MM_PER_IN,
            lead_out_kind: Lead::None,
            lead_out: 0.1 * MM_PER_IN,
            overcut: 0.0,
            use_material: true,
            kerf_comp: false,
            small_hole: 1.25 * MM_PER_IN,
            hole_speed_pct: 60.0,
            rapid: 600.0 * MM_PER_IN,
        }
    }
}

/// Kerf compensation for one cut.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Comp {
    Off,
    /// G41.1: the scrap is left of travel.
    Left,
    /// G42.1: the scrap is right of travel.
    Right,
}

/// One cut: pierce, lead-in, the contour, overcut and lead-out.
#[derive(Clone, Debug)]
pub struct Cut {
    pub placement: usize,
    pub hole: bool,
    pub small: bool,
    /// Where the torch fires.
    pub pierce: P2,
    /// From `pierce` to `contour.start`.
    pub lead_in: Vec<Seg>,
    pub contour: Contour,
    /// From `contour.start` along the contour again.
    pub overcut: Vec<Seg>,
    /// From the end of the overcut (or `contour.start`) into the scrap.
    pub lead_out: Vec<Seg>,
    pub comp: Comp,
}

fn polyline_len(pts: &[P2]) -> f64 {
    pts.windows(2)
        .map(|w| match w {
            [a, b] => dist(*a, *b),
            _ => 0.0,
        })
        .sum()
}

impl Cut {
    /// The whole torch path from the pierce as a polyline, with the indices where the contour starts and ends.
    pub fn polyline(&self) -> (Vec<P2>, usize, usize) {
        fn push(segs: &[Seg], pts: &mut Vec<P2>, at: &mut P2) {
            for s in segs {
                match *s {
                    Seg::Line(p) => pts.push(p),
                    Seg::Arc { to, c, ccw } => arc_points(*at, to, c, ccw, pts),
                }
                *at = s.end();
            }
        }
        let mut pts = vec![self.pierce];
        let mut at = self.pierce;
        push(&self.lead_in, &mut pts, &mut at);
        let a = pts.len() - 1;
        push(&self.contour.segs, &mut pts, &mut at);
        let b = pts.len() - 1;
        push(&self.overcut, &mut pts, &mut at);
        push(&self.lead_out, &mut pts, &mut at);
        (pts, a, b)
    }
    pub fn end(&self) -> P2 {
        self.lead_out.last().or(self.overcut.last()).map_or(self.contour.start, Seg::end)
    }
    /// Length of everything cut (leads included), mm.
    pub fn length(&self) -> f64 {
        polyline_len(&self.polyline().0)
    }
}

/// Unit travel direction at the start (`at_end` false) or end of `seg`, which begins at `from`.
fn tangent(from: P2, seg: &Seg, at_end: bool) -> P2 {
    match *seg {
        Seg::Line(p) => {
            let l = dist(from, p).max(1e-12);
            [(p[0] - from[0]) / l, (p[1] - from[1]) / l]
        }
        Seg::Arc { to, c, ccw } => {
            let q = if at_end { to } else { from };
            let a = (q[1] - c[1]).atan2(q[0] - c[0]);
            if ccw { [-a.sin(), a.cos()] } else { [a.sin(), -a.cos()] }
        }
    }
}

/// The first `len` mm of a contour, as segments from its start.
fn prefix(c: &Contour, len: f64) -> Vec<Seg> {
    let mut out = Vec::new();
    let mut left = len.min(c.length());
    let mut at = c.start;
    for s in &c.segs {
        if left <= 1e-6 {
            break;
        }
        match *s {
            Seg::Line(p) => {
                let l = dist(at, p);
                if l <= left {
                    out.push(*s);
                    left -= l;
                } else {
                    out.push(Seg::Line(lerp(at, p, left / l.max(1e-12))));
                    break;
                }
            }
            Seg::Arc { to, c: ctr, ccw } => {
                let r = dist(at, ctr);
                let l = r * sweep(at, to, ctr, ccw);
                if l <= left {
                    out.push(*s);
                    left -= l;
                } else {
                    let a = (at[1] - ctr[1]).atan2(at[0] - ctr[0]) + if ccw { 1.0 } else { -1.0 } * left / r.max(1e-12);
                    out.push(Seg::Arc { to: [ctr[0] + r * a.cos(), ctr[1] + r * a.sin()], c: ctr, ccw });
                    break;
                }
            }
        }
        at = s.end();
    }
    out
}

// --- Kerf offset ---------------------------------------------------------------------------------

/// One piece of an offset path: a line or an arc from `a` to `b`.
#[derive(Clone, Copy, Debug)]
enum El {
    Line { a: P2, b: P2 },
    Arc { a: P2, b: P2, c: P2, r: f64, ccw: bool },
}

impl El {
    fn a(&self) -> P2 {
        match *self {
            El::Line { a, .. } | El::Arc { a, .. } => a,
        }
    }
    fn b(&self) -> P2 {
        match *self {
            El::Line { b, .. } | El::Arc { b, .. } => b,
        }
    }
    fn set_a(&mut self, p: P2) {
        match self {
            El::Line { a, .. } | El::Arc { a, .. } => *a = p,
        }
    }
    fn set_b(&mut self, p: P2) {
        match self {
            El::Line { b, .. } | El::Arc { b, .. } => *b = p,
        }
    }
    fn seg(&self) -> Seg {
        match *self {
            El::Line { b, .. } => Seg::Line(b),
            El::Arc { b, c, ccw, .. } => Seg::Arc { to: b, c, ccw },
        }
    }
    fn tan(&self, at_end: bool) -> P2 {
        tangent(self.a(), &self.seg(), at_end)
    }
}

fn cross(a: P2, b: P2) -> f64 {
    a[0] * b[1] - a[1] * b[0]
}

/// Where two offset pieces (extended: whole lines, whole circles) cross, nearest `near`.
fn meet(p: &El, q: &El, near: P2) -> Option<P2> {
    let line_circle = |a: P2, b: P2, c: P2, r: f64| -> Vec<P2> {
        let d = [b[0] - a[0], b[1] - a[1]];
        let f = [a[0] - c[0], a[1] - c[1]];
        let qa = d[0] * d[0] + d[1] * d[1];
        let qb = 2.0 * (f[0] * d[0] + f[1] * d[1]);
        let qc = f[0] * f[0] + f[1] * f[1] - r * r;
        let disc = qb * qb - 4.0 * qa * qc;
        if qa < 1e-18 || disc < 0.0 {
            return Vec::new();
        }
        let s = disc.sqrt();
        [(-qb - s) / (2.0 * qa), (-qb + s) / (2.0 * qa)].iter().map(|t| [a[0] + d[0] * t, a[1] + d[1] * t]).collect()
    };
    let found: Vec<P2> = match (*p, *q) {
        (El::Line { a: a1, b: b1 }, El::Line { a: a2, b: b2 }) => {
            let (d1, d2) = ([b1[0] - a1[0], b1[1] - a1[1]], [b2[0] - a2[0], b2[1] - a2[1]]);
            let den = cross(d1, d2);
            if den.abs() < 1e-12 {
                Vec::new()
            } else {
                let t = cross([a2[0] - a1[0], a2[1] - a1[1]], d2) / den;
                vec![[a1[0] + d1[0] * t, a1[1] + d1[1] * t]]
            }
        }
        (El::Line { a, b }, El::Arc { c, r, .. }) | (El::Arc { c, r, .. }, El::Line { a, b }) => line_circle(a, b, c, r),
        (El::Arc { c: c1, r: r1, .. }, El::Arc { c: c2, r: r2, .. }) => {
            let d = dist(c1, c2);
            if d < 1e-12 || d > r1 + r2 || d < (r1 - r2).abs() {
                Vec::new()
            } else {
                let a = (r1 * r1 - r2 * r2 + d * d) / (2.0 * d);
                let h = (r1 * r1 - a * a).max(0.0).sqrt();
                let m = [c1[0] + a * (c2[0] - c1[0]) / d, c1[1] + a * (c2[1] - c1[1]) / d];
                let (ox, oy) = (h * (c2[1] - c1[1]) / d, h * (c2[0] - c1[0]) / d);
                vec![[m[0] + ox, m[1] - oy], [m[0] - ox, m[1] + oy]]
            }
        }
    };
    found.into_iter().filter(|x| x[0].is_finite() && x[1].is_finite()).min_by(|x, y| dist(*x, near).total_cmp(&dist(*y, near)))
}

/// A closed contour moved `d` mm to the left of travel (`left`) or the right, the way SheetCAM applies the kerf:
/// lines shift, arcs grow or shrink about their centres, outside corners are rounded with arcs of radius `d`
/// about the drawn corner, inside corners are trimmed where the moved pieces cross, and pieces the offset
/// swallows (a fillet smaller than `d`, a notch narrower than the kerf) drop out.
pub fn offset_contour(c: &Contour, d: f64, left: bool) -> Contour {
    if !(d > 1e-9) || !d.is_finite() || c.segs.is_empty() {
        return c.clone();
    }
    let mut dead = vec![false; c.segs.len()];
    for _pass in 0..12 {
        // Offset pieces, each with the drawn corner it ends at and its original sweep (arcs).
        let mut els: Vec<(usize, El, P2, f64)> = Vec::new();
        let mut from = c.start;
        for (i, s) in c.segs.iter().enumerate() {
            let to = s.end();
            if !dead.get(i).copied().unwrap_or(true) {
                match *s {
                    Seg::Line(p) => {
                        let l = dist(from, p);
                        if l > 1e-9 {
                            let t = [(p[0] - from[0]) / l, (p[1] - from[1]) / l];
                            let n = if left { [-t[1] * d, t[0] * d] } else { [t[1] * d, -t[0] * d] };
                            els.push((i, El::Line { a: [from[0] + n[0], from[1] + n[1]], b: [p[0] + n[0], p[1] + n[1]] }, to, 0.0));
                        }
                    }
                    Seg::Arc { to, c: ctr, ccw } => {
                        let r = dist(from, ctr);
                        let r2 = if ccw == left { r - d } else { r + d };
                        if r > 1e-9 && r2 > 1e-6 {
                            // Both ends onto the new radius (a drawn arc's ends can sit a hair off one circle).
                            let at = |p: P2| {
                                let l = dist(p, ctr).max(1e-12);
                                [ctr[0] + (p[0] - ctr[0]) * r2 / l, ctr[1] + (p[1] - ctr[1]) * r2 / l]
                            };
                            els.push((i, El::Arc { a: at(from), b: at(to), c: ctr, r: r2, ccw }, to, sweep(from, to, ctr, ccw)));
                        } else if let Some(x) = dead.get_mut(i) {
                            *x = true;
                        }
                    }
                }
            }
            from = to;
        }
        let n = els.len();
        if n == 0 {
            return c.clone();
        }
        // Join each piece to the next.
        let mut joins: Vec<Option<Seg>> = vec![None; n];
        for i in 0..n {
            let j = (i + 1) % n;
            let (Some(&(_, ei, v, _)), Some(&(_, ej, _, _))) = (els.get(i), els.get(j)) else { continue };
            let (e, s) = (ei.b(), ej.a());
            // Ends that already meet (within a fraction of LinuxCNC's arc tolerance) are simply joined.
            if dist(e, s) < 5e-4 {
                if let Some(x) = els.get_mut(j) {
                    x.1.set_a(e);
                }
                continue;
            }
            let (te, ts) = (ei.tan(true), ej.tan(false));
            let turn = cross(te, ts);
            let gap = if left { turn < 0.0 } else { turn > 0.0 };
            // Pieces meeting almost tangent (a drawing's rounding) are joined by the hair of a line between them.
            if turn.abs() < 0.035 && te[0] * ts[0] + te[1] * ts[1] > 0.0 {
                if let Some(x) = joins.get_mut(i) {
                    *x = Some(Seg::Line(s));
                }
            } else if gap && (dist(e, v) - d).abs() < 1e-3 * d.max(1.0) && (dist(s, v) - d).abs() < 1e-3 * d.max(1.0) {
                if let Some(x) = joins.get_mut(i) {
                    *x = Some(Seg::Arc { to: s, c: v, ccw: !left });
                }
            } else if let Some(x) = meet(&ei, &ej, v).filter(|x| dist(*x, v) < 50.0 * d) {
                if let Some(p) = els.get_mut(i) {
                    p.1.set_b(x);
                }
                if let Some(q) = els.get_mut(j) {
                    q.1.set_a(x);
                }
            } else if let Some(x) = joins.get_mut(i) {
                *x = Some(Seg::Line(s));
            }
        }
        // Pieces the trimming turned round are swallowed by the offset: drop them and join again.
        let mut again = false;
        if n > 1 {
            for (i, el, _, sw) in &els {
                let gone = match *el {
                    El::Line { a, b } => {
                        let o = c.segs.get(*i).map(|s| (if *i == 0 { c.start } else { c.segs.get(i - 1).map_or(c.start, Seg::end) }, s.end()));
                        o.is_none_or(|(oa, ob)| (b[0] - a[0]) * (ob[0] - oa[0]) + (b[1] - a[1]) * (ob[1] - oa[1]) <= 0.0)
                    }
                    El::Arc { a, b, c: ctr, ccw, .. } => dist(a, b) > 1e-7 && sweep(a, b, ctr, ccw) > sw + 1e-6,
                };
                if gone && let Some(x) = dead.get_mut(*i) {
                    *x = true;
                    again = true;
                }
            }
        }
        if again && dead.iter().any(|x| !x) {
            continue;
        }
        let start = els.first().map_or(c.start, |e| e.1.a());
        let mut segs = Vec::with_capacity(n * 2);
        for (k, (_, el, _, _)) in els.iter().enumerate() {
            if dist(el.a(), el.b()) > 1e-7 || matches!(el, El::Arc { .. }) && n == 1 {
                segs.push(el.seg());
            }
            if let Some(Some(j)) = joins.get(k) {
                segs.push(*j);
            }
        }
        if segs.is_empty() {
            return c.clone();
        }
        return Contour::new(start, segs);
    }
    c.clone()
}

/// The cuts for one sheet in order, SheetCAM's "keep parts together": each part's holes then its outside, the
/// next part the nearest to the torch, starting from the sheet origin. Every contour starts halfway along a
/// segment so leads meet the edge cleanly. `kerf` (mm) sets the shortest entry move kerf compensation needs.
pub fn toolpath(parts: &[Part], nest: &Nest, sheet: usize, post: &PostSettings, kerf: f64) -> Vec<Cut> {
    let mut left: Vec<usize> = (0..nest.placements.len()).filter(|i| nest.placements.get(*i).is_some_and(|p| p.sheet == sheet)).collect();
    let mut at: P2 = [0.0, 0.0];
    let mut cuts = Vec::new();
    let kerf = if kerf.is_finite() { kerf.max(0.0) } else { 0.0 };
    // Part in part: a part sitting in another's hole must be cut before that part, or the hole's slug drops
    // with it uncut. `inside[i]` lists the placements nested in placement i's holes.
    let bounds_of = |i: usize| -> Option<(P2, P2)> {
        let pl = nest.placements.get(i)?;
        Some(pl.contour(parts.get(pl.part)?.contours.first()?).bounds())
    };
    let mut inside: Vec<Vec<usize>> = vec![Vec::new(); nest.placements.len()];
    for &i in &left {
        let (Some(pl), Some(part)) = (nest.placements.get(i), nest.placements.get(i).and_then(|pl| parts.get(pl.part))) else { continue };
        for hole in part.contours.iter().skip(1) {
            let (hlo, hhi) = pl.contour(hole).bounds();
            for &j in &left {
                if j != i
                    && let Some((lo, hi)) = bounds_of(j)
                    && lo[0] >= hlo[0]
                    && lo[1] >= hlo[1]
                    && hi[0] <= hhi[0]
                    && hi[1] <= hhi[1]
                    && let Some(list) = inside.get_mut(i)
                {
                    list.push(j);
                }
            }
        }
    }
    while !left.is_empty() {
        // Nearest part by its box corner nearest the torch.
        let key = |i: &usize| -> f64 {
            nest.placements.get(*i).and_then(|pl| Some(pl.contour(parts.get(pl.part)?.contours.first()?))).map_or(f64::INFINITY, |c| {
                let (lo, hi) = c.bounds();
                let cx = at[0].clamp(lo[0], hi[0]);
                let cy = at[1].clamp(lo[1], hi[1]);
                dist(at, [cx, cy])
            })
        };
        // Only parts with nothing left to cut in their holes are ready.
        let ready = |i: &usize| inside.get(*i).is_none_or(|kids| kids.iter().all(|k| !left.contains(k)));
        let pool: Vec<usize> = (0..left.len()).filter(|k| left.get(*k).is_some_and(ready)).collect();
        let pool = if pool.is_empty() { (0..left.len()).collect() } else { pool };
        let Some(k) = pool.into_iter().min_by(|a, b| key(left.get(*a).unwrap_or(&0)).total_cmp(&key(left.get(*b).unwrap_or(&0)))) else { break };
        let pi = left.swap_remove(k);
        let Some(pl) = nest.placements.get(pi) else { continue };
        let Some(part) = parts.get(pl.part) else { continue };
        // Inside offset runs every contour the other way round, so the scrap stays on the left; Reverse
        // direction then swaps the travel (and the square edge) while the kerf stays on the same side.
        let flip = (part.side == Side::Inside) != part.reverse;
        let orient = |c: &Contour| {
            let c = pl.contour(c);
            if flip { c.reversed() } else { c }
        };
        let scrap_left = !part.reverse;
        // With no offset, Reverse cut side moves the leads to the other side of the line.
        let lead_left = if part.side == Side::OnLine { scrap_left != part.flip_lead } else { scrap_left };
        let comp = match (post.kerf_comp && part.side != Side::OnLine, scrap_left) {
            (false, _) => Comp::Off,
            (true, true) => Comp::Left,
            (true, false) => Comp::Right,
        };
        let mut holes: Vec<Contour> = part.contours.iter().skip(1).map(orient).collect();
        let outer = part.contours.first().map(orient);
        let mut order: Vec<(bool, Contour)> = Vec::new();
        while !holes.is_empty() {
            let Some(h) = (0..holes.len()).min_by(|a, b| {
                let d = |i: usize| holes.get(i).map_or(f64::INFINITY, |c| dist(at, c.start));
                d(*a).total_cmp(&d(*b))
            }) else {
                break;
            };
            let c = holes.swap_remove(h);
            at = c.start;
            order.push((true, c));
        }
        if let Some(o) = outer {
            order.push((false, o));
        }
        for (hole, c) in order {
            // Kerf in the CAM (SheetCAM's way): the torch path moves half a kerf toward the scrap side.
            let c = if post.kerf_comp || part.side == Side::OnLine { c } else { offset_contour(&c, kerf / 2.0, scrap_left) };
            let (lo, hi) = c.bounds();
            let size = (hi[0] - lo[0]).min(hi[1] - lo[1]);
            // Leads that stay inside a hole: at most a third of its width.
            let inside_hole = hole == (part.side != Side::Inside) && lead_left == scrap_left;
            let room = if inside_hole { size * 0.3 } else { f64::INFINITY };
            let lead_in = post.lead_in.max(0.0).min(room);
            let lead_out = post.lead_out.max(0.0).min(room);
            // Start halfway along the segment nearest the torch that is long enough for the lead.
            let mut best: Option<(f64, usize)> = None;
            let mut longest = (0.0, 0usize);
            for i in 0..c.segs.len() {
                let l = c.seg_len(i);
                if l > longest.0 {
                    longest = (l, i);
                }
                if l >= (lead_in * 0.8).max(0.5)
                    && let Some((m, _)) = c.mid_of(i)
                {
                    let d = dist(at, m);
                    if best.is_none_or(|b| d < b.0) {
                        best = Some((d, i));
                    }
                }
            }
            let c = c.restart_at_mid(best.map_or(longest.1, |b| b.1));
            let p0 = c.start;
            let d = c.segs.first().map_or([1.0, 0.0], |s| tangent(p0, s, false));
            let side = |d: P2| if lead_left { [-d[1], d[0]] } else { [d[1], -d[0]] };
            let n = side(d);
            let off = |p: P2, v: P2, k: f64| [p[0] + v[0] * k, p[1] + v[1] * k];
            // An entry move kerf compensation can start on (LinuxCNC needs a straight move longer than the radius).
            let entry = (kerf * 1.5).max(1.0).min(room.max(0.5));
            let comp_on = comp != Comp::Off;
            let (pierce, lead) = match post.lead_in_kind {
                Lead::Line if lead_in > 0.05 => (off(p0, n, lead_in.max(if comp_on { entry } else { 0.0 })), vec![Seg::Line(p0)]),
                Lead::Arc if lead_in > 0.05 => {
                    let ctr = off(p0, n, lead_in);
                    let a = off(ctr, d, -lead_in);
                    let arc = Seg::Arc { to: p0, c: ctr, ccw: lead_left };
                    if comp_on { (off(a, n, entry), vec![Seg::Line(a), arc]) } else { (a, vec![arc]) }
                }
                _ if comp_on => (off(p0, n, entry), vec![Seg::Line(p0)]),
                _ => (p0, Vec::new()),
            };
            let overcut = if post.overcut > 0.0 { prefix(&c, post.overcut) } else { Vec::new() };
            let (e, de) = match overcut.last() {
                Some(last) => {
                    let from = if overcut.len() >= 2 { overcut.get(overcut.len() - 2).map_or(p0, Seg::end) } else { p0 };
                    (last.end(), tangent(from, last, true))
                }
                None => (p0, d),
            };
            let ne = side(de);
            let out = match post.lead_out_kind {
                Lead::Line if lead_out > 0.05 => vec![Seg::Line(off(e, ne, lead_out))],
                Lead::Arc if lead_out > 0.05 => {
                    let ctr = off(e, ne, lead_out);
                    vec![Seg::Arc { to: off(ctr, de, lead_out), c: ctr, ccw: lead_left }]
                }
                _ => Vec::new(),
            };
            let small = hole && post.small_hole > 0.0 && (hi[0] - lo[0]).max(hi[1] - lo[1]) < post.small_hole;
            let cut = Cut { placement: pi, hole, small, pierce, lead_in: lead, contour: c, overcut, lead_out: out, comp };
            at = cut.end();
            cuts.push(cut);
        }
    }
    cuts
}

/// One step of the torch for the simulation.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MoveKind {
    Rapid,
    Pierce,
    Cut,
}

#[derive(Clone, Debug)]
pub struct Motion {
    pub kind: MoveKind,
    pub pts: Vec<P2>,
    pub seconds: f64,
}

/// The torch's moves for a sheet, timed: rapids at `post.rapid`, pierces at the tool's delay plus height finding,
/// cuts at the tool's feed (small holes slowed).
pub fn motions(cuts: &[Cut], tool: &Tool, post: &PostSettings) -> Vec<Motion> {
    let rapid = if post.rapid > 1.0 { post.rapid } else { 15_000.0 };
    let feed = if tool.feed > 1.0 { tool.feed } else { 1000.0 };
    let mut out = Vec::new();
    let mut at: P2 = [0.0, 0.0];
    for cut in cuts {
        out.push(Motion { kind: MoveKind::Rapid, pts: vec![at, cut.pierce], seconds: dist(at, cut.pierce) / rapid * 60.0 });
        out.push(Motion { kind: MoveKind::Pierce, pts: vec![cut.pierce], seconds: tool.pierce_delay.max(0.0) + 0.5 });
        let (pts, _, _) = cut.polyline();
        let speed = if cut.small { feed * post.hole_speed_pct.clamp(10.0, 100.0) / 100.0 } else { feed };
        let seconds = polyline_len(&pts) / speed * 60.0;
        out.push(Motion { kind: MoveKind::Cut, pts, seconds });
        at = cut.end();
    }
    out.push(Motion { kind: MoveKind::Rapid, pts: vec![at, [0.0, 0.0]], seconds: dist(at, [0.0, 0.0]) / rapid * 60.0 });
    out
}

/// The point `frac` (0–1) of the way along a polyline, and the points up to it.
pub fn along(pts: &[P2], frac: f64) -> (P2, Vec<P2>) {
    let mut want = polyline_len(pts) * frac.clamp(0.0, 1.0);
    let mut done = vec![pts.first().copied().unwrap_or([0.0, 0.0])];
    for w in pts.windows(2) {
        let [a, b] = w else { continue };
        let l = dist(*a, *b);
        if l >= want {
            let p = lerp(*a, *b, if l > 0.0 { want / l } else { 0.0 });
            done.push(p);
            return (p, done);
        }
        want -= l;
        done.push(*b);
    }
    (done.last().copied().unwrap_or([0.0, 0.0]), done)
}

/// A G-code number: fixed places, then trailing zeros (and a lone point) trimmed, as SheetCAM writes them.
fn gnum(v: f64, places: usize) -> String {
    let t = format!("{v:.places$}");
    let t = if t.contains('.') { t.trim_end_matches('0').trim_end_matches('.').to_string() } else { t };
    if t == "-0" { "0".into() } else { t }
}

/// Arcs that continue one another (same centre, radius and direction) merged into one move, so a drawn
/// circle is a single full-circle G3 as SheetCAM writes it. `from` is where `segs` start.
fn merge_arcs(from: P2, segs: &[Seg]) -> Vec<Seg> {
    let mut out: Vec<Seg> = Vec::with_capacity(segs.len());
    // Sweep so far of the arc being built, so a merge never passes a full turn.
    let (mut at, mut swept) = (from, 0.0);
    for s in segs {
        let next = match (out.last_mut(), *s) {
            (Some(Seg::Arc { to, c, ccw }), Seg::Arc { to: t2, c: c2, ccw: w2 })
                if *ccw == w2 && dist(*c, c2) < 1e-6 && (dist(at, c2) - dist(t2, c2)).abs() < 1e-6 =>
            {
                let more = sweep(at, t2, c2, w2);
                if swept + more <= TAU + 1e-6 {
                    *to = t2;
                    swept += more;
                    None
                } else {
                    Some(*s)
                }
            }
            _ => Some(*s),
        };
        if let Some(seg) = next {
            swept = match seg {
                Seg::Arc { to, c, ccw } => sweep(at, to, c, ccw),
                Seg::Line(_) => 0.0,
            };
            out.push(seg);
        }
        at = s.end();
    }
    out
}

/// QtPlasmaC G-code for one sheet, laid out like SheetCAM's LinuxCNC PlasmaC post: pre-amble, material
/// setup, then each cut as rapid, plasma start, lead-in, contour, lead-out and plasma end; post-amble and
/// statistics. With kerf compensation off (the default) the path itself carries the kerf offset.
pub fn gcode(parts: &[Part], nest: &Nest, sheet: usize, tool: &Tool, post: &PostSettings) -> String {
    use std::fmt::Write as _;
    let k = if post.inch { 1.0 / MM_PER_IN } else { 1.0 };
    let places = if post.inch { 5 } else { 3 };
    let n = |v: f64| gnum(v * k, places);
    let xy = |p: P2| format!("X{} Y{}", n(p[0]), n(p[1]));
    let clean = |t: &str| t.replace(['(', ')'], "");
    let cuts = toolpath(parts, nest, sheet, post, tool.kerf);
    let mut g = String::new();
    let _ = writeln!(g, ";  program: SolidCraft Plasma Nest, sheet {}", sheet + 1);
    let _ = writeln!(g, ";     tool: {} {}", tool.number, clean(&tool.name));
    let _ = writeln!(g, ";     kerf: {}{}", n(tool.kerf), if post.kerf_comp { " (compensated by QtPlasmaC)" } else { " (offset in the toolpath)" });
    let _ = writeln!(g, ";");
    let _ = writeln!(g, ";begin pre-amble");
    if post.inch {
        let _ = writeln!(g, " G20 (units: inches)");
    } else {
        let _ = writeln!(g, " G21 (units: metric)");
    }
    let _ = writeln!(g, " G40 (cutter compensation: off)");
    let _ = writeln!(g, " G90 (distance mode: absolute)");
    let _ = writeln!(g, " M52 P1 (adaptive feed: on)");
    let _ = writeln!(g, " M65 P2 (enable THC)");
    let _ = writeln!(g, " M65 P3 (enable torch)");
    let _ = writeln!(g, " M68 E3 Q0 (velocity 100%)");
    if post.inch {
        let _ = writeln!(g, " G64 P0.01 Q0.001 (tracking tolerances: 0.01\")");
    } else {
        let _ = writeln!(g, " G64 P0.25 Q0.025 (tracking tolerances: 0.25mm)");
    }
    let _ = writeln!(g, ";end pre-amble");
    let _ = writeln!(g, ";");
    let _ = writeln!(g, ";begin material setup");
    let _ = writeln!(g, " T0 M6 (select plasma tool)");
    let _ = writeln!(g, " G43 H0 (apply tool offsets)");
    if post.use_material {
        let _ = writeln!(g, " M190 P{} ({} material)", tool.number, clean(&tool.name));
        let _ = writeln!(g, " M66 P3 L3 Q2 (wait for valid change)");
        let _ = writeln!(g, " F#<_hal[plasmac.cut-feed-rate]>");
    } else {
        let _ = writeln!(g, " F{}", gnum(tool.feed * k, 1));
    }
    let _ = writeln!(g, ";end material setup");
    let mut last_part = usize::MAX;
    let mut length = 0.0;
    for cut in &cuts {
        if cut.placement != last_part {
            last_part = cut.placement;
            let name = nest.placements.get(cut.placement).and_then(|p| parts.get(p.part)).map_or("part", |p| p.name.as_str());
            let _ = writeln!(g, ";{}", clean(name));
        }
        length += cut.length();
        let _ = writeln!(g, " G0 {}", xy(cut.pierce));
        let _ = writeln!(g, " M3 $0 S1 (plasma start)");
        match cut.comp {
            Comp::Left => {
                let _ = writeln!(g, " G41.1 D#<_hal[plasmac_run.kerf-width-f]>");
            }
            Comp::Right => {
                let _ = writeln!(g, " G42.1 D#<_hal[plasmac_run.kerf-width-f]>");
            }
            Comp::Off => {}
        }
        if cut.small {
            let pct = post.hole_speed_pct.clamp(10.0, 100.0);
            let _ = writeln!(g, " M67 E3 Q{pct:.0} (velocity {pct:.0}%)");
        }
        let mut at = cut.pierce;
        let emit = |segs: &[Seg], at: &mut P2, g: &mut String| {
            for s in merge_arcs(*at, segs) {
                match s {
                    Seg::Line(p) => {
                        // A move too short to show in the output's places is left out.
                        if xy(p) != xy(*at) {
                            let _ = writeln!(g, " G1 {}", xy(p));
                        }
                    }
                    Seg::Arc { to, c, ccw } => {
                        let _ = writeln!(g, " {} {} I{} J{}", if ccw { "G3" } else { "G2" }, xy(to), n(c[0] - at[0]), n(c[1] - at[1]));
                    }
                }
                *at = s.end();
            }
        };
        emit(&cut.lead_in, &mut at, &mut g);
        emit(&cut.contour.segs, &mut at, &mut g);
        emit(&cut.overcut, &mut at, &mut g);
        emit(&cut.lead_out, &mut at, &mut g);
        if cut.small {
            let _ = writeln!(g, " M67 E3 Q0 (velocity 100%)");
        }
        if cut.comp != Comp::Off {
            let _ = writeln!(g, " G40 (cutter compensation: off)");
        }
        let _ = writeln!(g, " M5 $0 (plasma end)");
    }
    let _ = writeln!(g, " ;");
    let _ = writeln!(g, " ;begin post-amble");
    let _ = writeln!(g, " G40 (cutter compensation: off)");
    let _ = writeln!(g, " G90 (distance mode: absolute)");
    let _ = writeln!(g, " M65 P2 (enable THC)");
    let _ = writeln!(g, " M65 P3 (enable torch)");
    let _ = writeln!(g, " M68 E3 Q0 (velocity 100%)");
    let _ = writeln!(g, " M5 $-1 (backup stop)");
    let _ = writeln!(g, " ;end post-amble");
    let _ = writeln!(g, " ;");
    let _ = writeln!(g, " ;begin statistics");
    let _ = writeln!(g, " ;Pierces = {}", cuts.len());
    let unit = if post.inch { "\"" } else { "mm" };
    let _ = writeln!(g, " ;   Cuts = {},    Length = {:.2}{unit}", cuts.len(), length * k);
    let _ = writeln!(g, " ;end statistics");
    let _ = writeln!(g, " ;");
    let _ = writeln!(g, " M30 (end program)");
    g
}

#[cfg(test)]
mod tests {
    use super::*;
    use solidcraft_engine::geom::Vec2;
    use std::f64::consts::PI;

    fn v(x: f64, y: f64) -> Vec2 {
        Vec2 { x, y }
    }

    fn min_gap(a: &Contour, b: &Contour) -> f64 {
        let seg_d = |p: P2, a: P2, b: P2| {
            let d = [b[0] - a[0], b[1] - a[1]];
            let l2 = (d[0] * d[0] + d[1] * d[1]).max(1e-18);
            let t = (((p[0] - a[0]) * d[0] + (p[1] - a[1]) * d[1]) / l2).clamp(0.0, 1.0);
            dist(p, lerp(a, b, t))
        };
        let mut m = f64::INFINITY;
        for (x, y) in [(a, b), (b, a)] {
            for p in &x.pts {
                let n = y.pts.len();
                for i in 0..n {
                    m = m.min(seg_d(*p, y.pts[i], y.pts[(i + 1) % n]));
                }
            }
        }
        m
    }

    fn inside(p: P2, c: &Contour) -> bool {
        let n = c.pts.len();
        let mut odd = false;
        for i in 0..n {
            let (a, b) = (c.pts[i], c.pts[(i + 1) % n]);
            if (a[1] > p[1]) != (b[1] > p[1]) && p[0] < a[0] + (p[1] - a[1]) / (b[1] - a[1]) * (b[0] - a[0]) {
                odd = !odd;
            }
        }
        odd
    }

    #[test]
    fn lines_and_arcs_chain_into_a_rounded_square_with_a_hole() {
        let g = vec![
            Geom2::Line(v(10.0, 0.0), v(90.0, 0.0)),
            Geom2::Arc { c: v(90.0, 10.0), a: v(90.0, 0.0), b: v(100.0, 10.0) },
            Geom2::Line(v(100.0, 90.0), v(100.0, 10.0)), // reversed piece
            Geom2::Arc { c: v(90.0, 90.0), a: v(100.0, 90.0), b: v(90.0, 100.0) },
            Geom2::Line(v(90.0, 100.0), v(10.0, 100.0)),
            Geom2::Arc { c: v(10.0, 90.0), a: v(10.0, 100.0), b: v(0.0, 90.0) },
            Geom2::Line(v(0.0, 90.0), v(0.0, 10.0)),
            Geom2::Arc { c: v(10.0, 10.0), a: v(0.0, 10.0), b: v(10.0, 0.0) },
            Geom2::Circle(v(50.0, 50.0), 10.0),
        ];
        let p = part_from_geoms("sq", &g).unwrap();
        assert_eq!(p.contours.len(), 2);
        assert!((p.width - 100.0).abs() < 1e-9);
        let arcs = p.contours[0].segs.iter().filter(|s| matches!(s, Seg::Arc { .. })).count();
        assert_eq!(arcs, 4, "corner arcs stay arcs");
        let expect_len = 4.0 * 80.0 + 2.0 * PI * 10.0;
        assert!((p.contours[0].length() - expect_len).abs() < 1e-6);
        assert!(p.contours[0].signed_area() < 0.0, "outer is clockwise");
        assert!(p.contours[1].signed_area() > 0.0, "hole is counter-clockwise");
        let area = 100.0 * 100.0 - (4.0 - PI) * 100.0 - PI * 100.0;
        assert!((p.area() - area).abs() < 20.0);
    }

    #[test]
    fn open_drawings_are_refused_not_crashed() {
        assert!(part_from_geoms("x", &[Geom2::Line(v(0.0, 0.0), v(1.0, 0.0))]).is_err());
        assert!(part_from_dxf("x", "garbage").is_err());
        assert!(part_from_geoms("x", &[]).is_err());
    }

    fn settings(step: f64) -> NestSettings {
        NestSettings {
            sheet_w: 96.0 * MM_PER_IN,
            sheet_h: 48.0 * MM_PER_IN,
            spacing: 6.0,
            margin: 12.0,
            rotation_step: step,
            max_sheets: 10,
            part_in_part: true,
        }
    }

    #[test]
    fn true_shape_nest_keeps_the_spacing_and_the_sheet() {
        let parts = sample_parts();
        let s = settings(90.0);
        let n = nest(&parts, &s);
        let total: u32 = parts.iter().map(|p| p.qty).sum();
        assert_eq!(n.placements.len(), total as usize);
        assert!(n.unplaced.is_empty());
        let outers: Vec<(usize, Contour, Vec<Contour>)> = n
            .placements
            .iter()
            .map(|pl| {
                let p = &parts[pl.part];
                (pl.sheet, pl.contour(&p.contours[0]), p.contours.iter().skip(1).map(|c| pl.contour(c)).collect())
            })
            .collect();
        for (sheet, o, _) in &outers {
            let (lo, hi) = o.bounds();
            assert!(lo[0] >= s.margin - 1e-6 && lo[1] >= s.margin - 1e-6, "{sheet} {lo:?}");
            assert!(hi[0] <= s.sheet_w - s.margin + 1e-6 && hi[1] <= s.sheet_h - s.margin + 1e-6);
        }
        let mut nested_in_holes = 0;
        for (i, (sa, a, holes_a)) in outers.iter().enumerate() {
            for (sb, b, _) in outers.iter().skip(i + 1) {
                if sa != sb {
                    continue;
                }
                let gap = min_gap(a, b);
                assert!(gap >= s.spacing - 1e-6, "parts {gap:.2} mm apart");
                // Overlap without touching edges: one inside the other's solid (not in a hole) is a failure.
                let b_in_a = inside(b.pts[0], a);
                if b_in_a {
                    assert!(holes_a.iter().any(|h| inside(b.pts[0], h)), "a part sits on another's solid area");
                    nested_in_holes += 1;
                }
                assert!(!inside(a.pts[0], b) || !b_in_a);
            }
        }
        assert!(nested_in_holes > 0, "washers fill the plates' centre holes");
    }

    #[test]
    fn true_shape_beats_boxes() {
        // Right triangles: rotated 180° they pair into squares that boxes cannot share.
        let tri = Part::finish("tri", vec![Contour::from_points(vec![[0.0, 0.0], [200.0, 0.0], [0.0, 200.0]])], 0).unwrap();
        let parts = vec![Part { qty: 40, ..tri }];
        let s = NestSettings { sheet_w: 1000.0, sheet_h: 1000.0, spacing: 5.0, margin: 5.0, rotation_step: 90.0, max_sheets: 4, part_in_part: true };
        let n = nest(&parts, &s);
        assert_eq!(n.placements.len(), 40);
        // Boxes of 205 mm fit 4 x 4 = 16 per sheet; true shape needs fewer sheets than 3.
        assert!(n.sheets <= 2, "{} sheets", n.sheets);
    }

    /// A customer's wheel center (inches, as G-code moves: a start point then lines and arcs). SheetCAM
    /// nests four on a 24" x 24" sheet at 1/8" spacing; best fit must too.
    const WHEEL_HOLE: &str = "G1 X8.29776 Y6.48243
    G3 X6.61026 Y8.16993 I-1.68750 J0.00000
    G3 X6.61026 Y4.79493 I0.00000 J-1.68750
    G3 X8.29776 Y6.48243 I0.00000 J1.68750";
    const WHEEL_OUTER: &str = "G1 X9.36026 Y6.48243
    G2 X9.31597 Y5.99090 I-2.75000 J0.00000
    G3 X9.49666 Y5.51022 I0.49195 J-0.08937
    G1 X12.36823 Y3.22607
    G2 X11.48656 Y2.01254 I-5.75798 J3.25636
    G1 X8.42683 Y4.03773
    G3 X7.91385 Y4.06104 I-0.27597 J-0.41694
    G2 X6.97889 Y3.75725 I-1.30359 J2.42139
    G3 X6.57758 Y3.43687 I0.06702 J-0.49549
    G1 X5.29258 Y0.00000
    G2 X3.86600 Y0.46353 I1.31767 J6.48243
    G1 X4.84656 Y3.99931
    G3 X4.71021 Y4.49439 I-0.48182 J0.13362
    G2 X4.13237 Y5.28972 I1.90005 J1.98804
    G3 X3.70365 Y5.57238 I-0.45053 J-0.21686
    G1 X0.03791 Y5.73243
    G2 X0.03791 Y7.23243 I6.57235 J0.75000
    G1 X3.70365 Y7.39248
    G3 X4.13237 Y7.67515 I-0.02181 J0.49952
    G2 X4.71021 Y8.47048 I2.47789 J-1.19272
    G3 X4.84656 Y8.96556 I-0.34546 J0.36146
    G1 X3.86600 Y12.50134
    G2 X5.29258 Y12.96487 I2.74426 J-6.01891
    G1 X6.57758 Y9.52800
    G3 X6.97889 Y9.20762 I0.46834 J0.17510
    G2 X7.91385 Y8.90383 I-0.36863 J-2.72518
    G3 X8.42683 Y8.92714 I0.23702 J0.44025
    G1 X11.48656 Y10.95233
    G2 X12.36823 Y9.73880 I-4.87630 J-4.46989
    G1 X9.49666 Y7.45465
    G3 X9.31597 Y6.97397 I0.31126 J-0.39130
    G2 X9.36026 Y6.48243 I-2.70571 J-0.49154";

    fn contour_from_moves(text: &str) -> Contour {
        let val = |l: &str, c: char| l.split_whitespace().find_map(|w| w.strip_prefix(c)?.parse::<f64>().ok()).unwrap() * 25.4;
        let mut lines = text.lines().map(str::trim).filter(|l| !l.is_empty());
        let first = lines.next().unwrap();
        let start = [val(first, 'X'), val(first, 'Y')];
        let mut at = start;
        let mut segs = Vec::new();
        for l in lines {
            let p = [val(l, 'X'), val(l, 'Y')];
            segs.push(if l.starts_with("G1") {
                Seg::Line(p)
            } else {
                Seg::Arc { to: p, c: [at[0] + val(l, 'I'), at[1] + val(l, 'J')], ccw: l.starts_with("G3") }
            });
            at = p;
        }
        Contour::new(start, segs)
    }

    #[test]
    fn best_fit_nests_four_wheel_centers_on_a_24_inch_sheet() {
        let mut wheel = part_from_contours("wheel", vec![contour_from_moves(WHEEL_OUTER), contour_from_moves(WHEEL_HOLE)]).unwrap();
        wheel.qty = 4;
        let parts = vec![wheel];
        let s = NestSettings {
            sheet_w: 24.0 * 25.4,
            sheet_h: 24.0 * 25.4,
            spacing: 0.125 * 25.4,
            margin: 0.0,
            rotation_step: BEST_FIT,
            max_sheets: 4,
            part_in_part: true,
        };
        // Plain quarter turns only fit three.
        let quarter = nest(&parts, &NestSettings { rotation_step: 90.0, ..s });
        assert!(quarter.sheets > 1);
        let n = nest_search(&parts, &s, web_time::Duration::from_secs(300));
        assert_eq!((n.placements.len(), n.sheets), (4, 1), "angles {:?}", n.placements.iter().map(|p| p.angle_deg).collect::<Vec<_>>());
        for i in 0..n.placements.len() {
            let (hits, off) = contacts(&parts, &n, i, &s);
            assert!(hits.is_empty() && !off, "placement {i} touches {hits:?} off={off}");
        }
    }

    #[test]
    fn best_fit_rotation_nests_without_contact_and_squares_a_tilted_part() {
        // A rectangle drawn at 30 degrees: best fit turns it square to the sheet.
        let (c, sn) = (30f64.to_radians().cos(), 30f64.to_radians().sin());
        let rot = |x: f64, y: f64| [c * x - sn * y, sn * x + c * y];
        let tilted =
            Part::finish("tilted", vec![Contour::from_points(vec![rot(0.0, 0.0), rot(300.0, 0.0), rot(300.0, 100.0), rot(0.0, 100.0)])], 0).unwrap();
        let angles = best_fit_angles(&tilted);
        assert!(angles.iter().any(|a| (a - 330.0).abs() < 0.5 || (a - 150.0).abs() < 0.5), "{angles:?}");
        let mut parts = sample_parts();
        parts.push(Part { qty: 6, ..tilted });
        let length = |n: &Nest| (0..n.sheets).map(|k| used_length(&parts, n, k)).sum::<f64>();
        // Best fit is never worse than quarter turns, and every layout keeps the spacing.
        let best = nest(&parts, &NestSettings { rotation_step: BEST_FIT, ..settings(90.0) });
        let quarter = nest(&parts, &settings(90.0));
        assert!((best.sheets, length(&best)) <= (quarter.sheets, length(&quarter) + 1e-6));
        let s = NestSettings { rotation_step: HULL_ANGLES, ..settings(90.0) };
        let n = nest(&parts, &s);
        assert!(n.unplaced.is_empty());
        for nn in [&n, &best] {
            for i in 0..nn.placements.len() {
                let (hits, off) = contacts(&parts, nn, i, &s);
                assert!(hits.is_empty() && !off, "placement {i} touches {hits:?} off={off}");
            }
        }
        // The tilted part was squared up: its placed box is close to 300 x 100.
        let ti = parts.len() - 1;
        let pl = n.placements.iter().find(|p| p.part == ti).unwrap();
        let (lo, hi) = pl.contour(&parts[ti].contours[0]).bounds();
        let (w, h) = (hi[0] - lo[0], hi[1] - lo[1]);
        assert!((w.max(h) - 300.0).abs() < 1.0 && (w.min(h) - 100.0).abs() < 1.0, "{w} x {h}");
    }

    #[test]
    fn manual_moves_report_contact_and_leaving_the_sheet() {
        let parts = sample_parts();
        let s = settings(90.0);
        let mut n = nest(&parts, &s);
        assert_eq!(contacts(&parts, &n, 0, &s), (Vec::new(), false));
        // Drop part 0 onto part 1.
        let (a, b) = (n.placements[0], n.placements[1]);
        let pa = a.contour(&parts[a.part].contours[0]).bounds().0;
        let pb = b.contour(&parts[b.part].contours[0]).bounds().0;
        n.placements[0].tx += pb[0] - pa[0] + 5.0;
        n.placements[0].ty += pb[1] - pa[1] + 5.0;
        assert!(contacts(&parts, &n, 0, &s).0.contains(&1));
        // Push it off the sheet.
        n.placements[0].tx -= 5000.0;
        assert!(contacts(&parts, &n, 0, &s).1);
    }

    #[test]
    fn hostile_nest_inputs_never_panic() {
        let parts = sample_parts();
        for s in [
            NestSettings { sheet_w: 0.0, sheet_h: 0.0, spacing: 0.0, margin: 0.0, rotation_step: 90.0, max_sheets: 0, part_in_part: true },
            NestSettings {
                sheet_w: f64::NAN,
                sheet_h: 10.0,
                spacing: -5.0,
                margin: 1e9,
                rotation_step: f64::NAN,
                max_sheets: 1,
                part_in_part: false,
            },
            NestSettings { sheet_w: 10.0, sheet_h: 10.0, spacing: 0.0, margin: 0.0, rotation_step: 1.0, max_sheets: 3, part_in_part: true },
        ] {
            let _ = nest(&parts, &s);
        }
        let mut huge = sample_parts();
        huge[0].qty = u32::MAX;
        let n = nest(
            &huge,
            &NestSettings { sheet_w: 100.0, sheet_h: 100.0, spacing: 1.0, margin: 0.0, rotation_step: 90.0, max_sheets: 2, part_in_part: true },
        );
        assert!(n.truncated > 0);
    }

    #[test]
    fn the_powermax_library_loads() {
        let tools = default_tools();
        assert_eq!(tools.len(), 92);
        let t = tools.iter().find(|t| t.name == "65A 1/4 Steel").unwrap();
        assert_eq!(t.number, 61);
        assert_eq!(t.amps, 65);
        assert_eq!(t.metal, Metal::Steel);
        assert!((t.thickness_in.unwrap() - 0.25).abs() < 1e-9);
        assert!((t.kerf - 1.778).abs() < 1e-9 && (t.feed - 2286.0).abs() < 1e-9 && (t.volts - 127.0).abs() < 1e-9);
        let g = tools.iter().find(|t| t.name == "45A FineCut 12GA Stainless").unwrap();
        assert_eq!(g.metal, Metal::Stainless);
        assert!((g.thickness_in.unwrap() - 0.1094).abs() < 1e-9);
        let oxy = tools.iter().find(|t| t.name.starts_with("OXY 1 1/2")).unwrap();
        assert!(oxy.oxy && (oxy.thickness_in.unwrap() - 1.5).abs() < 1e-9);
        let cfg = qtplasmac_materials(&tools, true);
        assert!(cfg.contains("[MATERIAL_NUMBER_61]") && cfg.contains("CUT_SPEED          = 90.0"));
        assert!(parse_tools("[[[\n=\n[Tool0]\nType=PlasmaTool\nFeed\\ rate=abc").len() == 1);
    }

    #[test]
    fn quote_lines_add_up_and_thicker_steel_costs_more() {
        let parts = sample_parts();
        let tools = default_tools();
        let pr = Pricing::default();
        let thin = quote(&parts, tools.iter().find(|t| t.name == "45A 12GA Steel").unwrap(), &pr, None);
        let thick = quote(&parts, tools.iter().find(|t| t.name == "65A 1/2 Steel").unwrap(), &pr, None);
        let sum: f64 = thin.lines.iter().map(|l| l.total).sum();
        assert!((sum - thin.total).abs() < 1e-6);
        assert!(thick.total > thin.total);
        assert!(thick.weight_lb > thin.weight_lb);
        // Price per pierce and per cut inch add exactly their share (before markup).
        let rates = Pricing { per_pierce: 0.5, per_inch: 0.1, markup_pct: 0.0, ..pr.clone() };
        let base = quote(&parts, tools.iter().find(|t| t.name == "45A 12GA Steel").unwrap(), &Pricing { markup_pct: 0.0, ..pr.clone() }, None);
        let with = quote(&parts, tools.iter().find(|t| t.name == "45A 12GA Steel").unwrap(), &rates, None);
        let expect = base.pierces as f64 * 0.5 + base.cut_in * 0.1;
        assert!((with.total - base.total - expect).abs() < 1e-6);
        assert!((with.lines.iter().map(|l| l.total).sum::<f64>() - with.total).abs() < 1e-6);
    }

    /// LinuxCNC's rules: every arc's centre is equidistant from its ends, and the move after G41.1/G42.1 is a
    /// straight line.
    fn check_gcode(g: &str) {
        let mut at = [0.0f64, 0.0];
        let mut after_comp = false;
        for line in g.lines().map(str::trim_start).filter(|l| !l.starts_with(';')) {
            let val = |c: char| line.split_whitespace().find_map(|w| w.strip_prefix(c)?.parse::<f64>().ok());
            let motion = line.starts_with("G0 ") || line.starts_with("G1 ") || line.starts_with("G2 ") || line.starts_with("G3 ");
            if after_comp && motion {
                assert!(line.starts_with("G1 "), "first move after compensation must be straight: {line}");
                after_comp = false;
            }
            if line.starts_with("G41.1") || line.starts_with("G42.1") {
                after_comp = true;
            }
            if line.starts_with("G2 ") || line.starts_with("G3 ") {
                let (x, y, i, j) = (val('X').unwrap(), val('Y').unwrap(), val('I').unwrap(), val('J').unwrap());
                let c = [at[0] + i, at[1] + j];
                assert!((dist(at, c) - dist([x, y], c)).abs() < 0.0005, "{line}");
            }
            if let (Some(x), Some(y)) = (val('X'), val('Y')) {
                at = [x, y];
            }
        }
    }

    /// Furthest and nearest any point of `a` lies from the outline of `b`.
    fn gap_range(a: &Contour, b: &Contour) -> (f64, f64) {
        let seg_d = |p: P2, a: P2, b: P2| {
            let d = [b[0] - a[0], b[1] - a[1]];
            let l2 = (d[0] * d[0] + d[1] * d[1]).max(1e-18);
            let t = (((p[0] - a[0]) * d[0] + (p[1] - a[1]) * d[1]) / l2).clamp(0.0, 1.0);
            dist(p, lerp(a, b, t))
        };
        // The outline of `b` finely divided, so chord sag does not count as distance.
        let mut fine = vec![b.start];
        let mut at = b.start;
        for s in &b.segs {
            if let Seg::Arc { to, c, ccw } = *s {
                let (r, sw, a0) = (dist(at, c), sweep(at, to, c, ccw), (at[1] - c[1]).atan2(at[0] - c[0]));
                let k = (sw.to_degrees() * 20.0).ceil() as usize;
                fine.extend((1..k).map(|i| {
                    let a = a0 + if ccw { 1.0 } else { -1.0 } * sw * i as f64 / k as f64;
                    [c[0] + r * a.cos(), c[1] + r * a.sin()]
                }));
            }
            fine.push(s.end());
            at = s.end();
        }
        let n = fine.len();
        let ds: Vec<f64> = a.pts.iter().map(|p| (0..n).map(|i| seg_d(*p, fine[i], fine[(i + 1) % n])).fold(f64::INFINITY, f64::min)).collect();
        (ds.iter().copied().fold(f64::INFINITY, f64::min), ds.iter().copied().fold(0.0, f64::max))
    }

    #[test]
    fn kerf_offset_keeps_half_a_kerf_off_every_edge_on_the_scrap_side() {
        let tool = default_tools().into_iter().find(|t| t.number == 44).unwrap();
        let half = tool.kerf / 2.0;
        let wheel = part_from_contours("wheel", vec![contour_from_moves(WHEEL_OUTER), contour_from_moves(WHEEL_HOLE)]).unwrap();
        let mut parts = sample_parts();
        parts.push(wheel);
        for part in &parts {
            for (i, c) in part.contours.iter().enumerate() {
                for left in [true, false] {
                    let o = offset_contour(c, half, left);
                    let (lo, hi) = gap_range(&o, c);
                    assert!((lo - half).abs() < 0.03 && (hi - half).abs() < 0.03, "{} contour {i} left={left}: {lo}..{hi} vs {half}", part.name);
                    // Joined end to end, and every arc a true arc.
                    let mut at = o.start;
                    for s in &o.segs {
                        if let Seg::Arc { to, c, .. } = *s {
                            assert!((dist(at, c) - dist(to, c)).abs() < 1e-3, "{}: arc ends off its circle", part.name);
                        }
                        at = s.end();
                    }
                    assert!(dist(at, o.start) < 1e-6, "{} contour {i} is closed", part.name);
                    // Left of a clockwise outline is outside it (bigger); left of a counter-clockwise hole is inside it.
                    let grows = left == (c.signed_area() < 0.0);
                    assert_eq!(o.signed_area().abs() > c.signed_area().abs(), grows, "{} contour {i} left={left}", part.name);
                }
            }
        }
    }

    #[test]
    fn toolpath_cuts_outside_outsides_and_inside_holes_by_half_the_kerf() {
        let tool = default_tools().into_iter().find(|t| t.number == 44).unwrap();
        let half = tool.kerf / 2.0;
        let mut wheel = part_from_contours("wheel", vec![contour_from_moves(WHEEL_OUTER), contour_from_moves(WHEEL_HOLE)]).unwrap();
        wheel.qty = 1;
        for side in [Side::Outside, Side::Inside, Side::OnLine] {
            for reverse in [false, true] {
                let parts = vec![Part { side, reverse, ..wheel.clone() }];
                let n = nest(&parts, &settings(90.0));
                let pl = &n.placements[0];
                for cut in toolpath(&parts, &n, 0, &PostSettings::default(), tool.kerf) {
                    let drawn = pl.contour(&parts[0].contours[if cut.hole { 1 } else { 0 }]);
                    let (lo, hi) = gap_range(&cut.contour, &drawn);
                    let want = if side == Side::OnLine { 0.0 } else { half };
                    assert!((lo - want).abs() < 0.03 && (hi - want).abs() < 0.03, "{side:?} hole={}: {lo}..{hi}", cut.hole);
                    // Outside: the outline grows and the hole shrinks; Inside the other way round.
                    let bigger = cut.contour.signed_area().abs() > drawn.signed_area().abs();
                    match side {
                        Side::Outside => assert_eq!(bigger, !cut.hole, "outside, hole={}", cut.hole),
                        Side::Inside => assert_eq!(bigger, cut.hole, "inside, hole={}", cut.hole),
                        Side::OnLine => {}
                    }
                }
            }
        }
        // As SheetCAM posts it: the round hole is one full-circle G3 of the hole's radius less half the kerf.
        let parts = vec![wheel];
        let n = nest(&parts, &settings(90.0));
        let g = gcode(&parts, &n, 0, &tool, &PostSettings::default());
        check_gcode(&g);
        let r = 1.6875 - half / MM_PER_IN;
        let full = g.lines().map(str::trim).filter(|l| l.starts_with("G3 ")).filter(|l| {
            let v = |c: char| l.split_whitespace().find_map(|w| w.strip_prefix(c)?.parse::<f64>().ok()).unwrap_or(0.0);
            ((v('I').powi(2) + v('J').powi(2)).sqrt() - r).abs() < 2e-4
        });
        assert_eq!(full.count(), 1, "{g}");
    }

    #[test]
    fn gcode_uses_arcs_material_and_kerf_comp() {
        let parts = sample_parts();
        let n = nest(&parts, &settings(90.0));
        let tool = default_tools().into_iter().find(|t| t.number == 61).unwrap();
        let g = gcode(&parts, &n, 0, &tool, &PostSettings::default());
        let contours: usize = n.placements.iter().filter(|p| p.sheet == 0).map(|p| parts[p.part].contours.len()).sum();
        assert_eq!(g.matches("M3 $0 S1 (plasma start)").count(), contours);
        assert_eq!(g.matches("M5 $0 (plasma end)").count(), contours);
        // SheetCAM's layout: the kerf is in the path, so no G41.1; material setup; backup stop and M30.
        assert_eq!(g.matches("G41.1").count() + g.matches("G42.1").count(), 0);
        assert!(g.contains(" G20 (units: inches)\n G40 (cutter compensation: off)\n G90 (distance mode: absolute)"));
        assert!(g.contains(" T0 M6 (select plasma tool)\n G43 H0 (apply tool offsets)\n M190 P61 ("));
        assert!(g.contains(" M66 P3 L3 Q2 (wait for valid change)\n F#<_hal[plasmac.cut-feed-rate]>"));
        assert!(g.contains(&format!(" ;Pierces = {contours}")));
        assert!(g.contains(" M5 $-1 (backup stop)"));
        assert!(g.trim_end().ends_with(" M30 (end program)"));
        assert!(!g.contains("G0 X0 Y0"), "no return to the origin");
        assert!(g.contains("G2 ") && g.contains("G3 "), "circles and corners post as arcs");
        assert!(g.contains("M67 E3 Q60 (velocity 60%)") && g.contains("M67 E3 Q0 (velocity 100%)"), "small holes slow down");
        assert!(
            !g.lines()
                .any(|l| l.split_whitespace().any(|w| w.len() > 1 && w.ends_with('0') && w.contains('.') && w.starts_with(['X', 'Y', 'I', 'J']))),
            "trailing zeros trimmed"
        );
        check_gcode(&g);
        // Controller compensation, the old way, is still there behind the setting.
        let g41 = gcode(&parts, &n, 0, &tool, &PostSettings { kerf_comp: true, ..PostSettings::default() });
        assert_eq!(g41.matches("G41.1").count(), contours);
        check_gcode(&g41);
        // A rotated nest still posts true arcs.
        let n45 = nest(&parts, &settings(45.0));
        let g45 = gcode(&parts, &n45, 0, &tool, &PostSettings::default());
        assert!(g45.contains("G2 "));
        check_gcode(&g45);
    }

    #[test]
    fn every_lead_and_side_posts_valid_gcode_and_pierces_in_the_scrap() {
        let tool = default_tools().into_iter().find(|t| t.number == 61).unwrap();
        for side in [Side::Outside, Side::Inside, Side::OnLine] {
            for reverse in [false, true] {
                for (lead_in_kind, lead_out_kind, overcut) in
                    [(Lead::Arc, Lead::Arc, 3.0), (Lead::Line, Lead::Line, 0.0), (Lead::None, Lead::None, 0.0), (Lead::Arc, Lead::None, 0.0)]
                {
                    let mut parts = sample_parts();
                    for p in &mut parts {
                        p.side = side;
                        p.reverse = reverse;
                    }
                    let n = nest(&parts, &settings(90.0));
                    for kerf_comp in [false, true] {
                        let post = PostSettings { lead_in_kind, lead_out_kind, overcut, kerf_comp, ..PostSettings::default() };
                        let g = gcode(&parts, &n, 0, &tool, &post);
                        check_gcode(&g);
                        let contours: usize = n.placements.iter().filter(|p| p.sheet == 0).map(|p| parts[p.part].contours.len()).sum();
                        let (left, right) = (g.matches("G41.1").count(), g.matches("G42.1").count());
                        match (side, reverse) {
                            _ if !kerf_comp => assert_eq!(left + right, 0),
                            (Side::OnLine, _) => assert_eq!(left + right, 0),
                            (_, false) => assert_eq!((left, right), (contours, 0)),
                            (_, true) => assert_eq!((left, right), (0, contours)),
                        }
                        for cut in toolpath(&parts, &n, 0, &post, tool.kerf) {
                            // No lead-in means none: the torch fires on the path itself.
                            if lead_in_kind == Lead::None && !kerf_comp {
                                assert!(cut.lead_in.is_empty() && dist(cut.pierce, cut.contour.start) < 1e-9);
                                continue;
                            }
                            // Direction: outsides clockwise unless Inside xor Reverse.
                            let flip = (side == Side::Inside) != reverse;
                            let ccw = cut.contour.signed_area() > 0.0;
                            assert_eq!(ccw, cut.hole != flip, "{side:?} reverse={reverse} hole={}", cut.hole);
                            if lead_in_kind == Lead::None && side == Side::OnLine {
                                continue;
                            }
                            // The pierce is on the scrap side of the line.
                            let inside_line = inside(cut.pierce, &cut.contour);
                            let scrap_inside = cut.hole != (side == Side::Inside);
                            if side != Side::OnLine {
                                assert_eq!(inside_line, scrap_inside, "{side:?} reverse={reverse} hole={} {lead_in_kind:?}", cut.hole);
                            }
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn parts_in_holes_are_cut_before_the_part_around_them() {
        let parts = sample_parts();
        let n = nest(&parts, &settings(90.0));
        let tool = default_tools().into_iter().find(|t| t.number == 61).unwrap();
        let cuts = toolpath(&parts, &n, 0, &PostSettings::default(), tool.kerf);
        let washer = parts.iter().position(|p| p.name.starts_with("washer")).unwrap();
        let mut checked = 0;
        for (wi, w) in n.placements.iter().enumerate().filter(|(_, p)| p.part == washer && p.sheet == 0) {
            let (lo, hi) = w.contour(&parts[washer].contours[0]).bounds();
            let last_washer_cut = cuts.iter().rposition(|c| c.placement == wi).unwrap();
            for (ci, c) in cuts.iter().enumerate().filter(|(_, c)| c.hole && c.placement != wi) {
                let (hlo, hhi) = c.contour.bounds();
                if lo[0] >= hlo[0] && lo[1] >= hlo[1] && hi[0] <= hhi[0] && hi[1] <= hhi[1] {
                    assert!(last_washer_cut < ci, "the washer in a hole is cut after the hole");
                    checked += 1;
                }
            }
        }
        assert!(checked > 0, "some washer sits in a plate's hole");
    }

    #[test]
    fn simulation_motions_cover_the_whole_program() {
        let parts = sample_parts();
        let n = nest(&parts, &settings(90.0));
        let tool = default_tools().into_iter().find(|t| t.number == 61).unwrap();
        let post = PostSettings::default();
        let cuts = toolpath(&parts, &n, 0, &post, tool.kerf);
        let m = motions(&cuts, &tool, &post);
        assert_eq!(m.len(), cuts.len() * 3 + 1);
        assert_eq!(m.iter().filter(|x| x.kind == MoveKind::Pierce).count(), cuts.len());
        let total: f64 = m.iter().map(|x| x.seconds).sum();
        assert!(total > 60.0 && total < 3600.0, "{total}");
        let cut = m.iter().find(|x| x.kind == MoveKind::Cut).unwrap();
        let (end, done) = along(&cut.pts, 1.0);
        assert!(dist(end, *cut.pts.last().unwrap()) < 1e-9 && done.len() == cut.pts.len());
        let (mid, _) = along(&cut.pts, 0.5);
        assert!(mid[0].is_finite());
        assert_eq!(along(&[], 0.5).1.len(), 1);
    }
}
