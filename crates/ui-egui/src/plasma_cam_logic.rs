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
        Ok(Part { name: name.to_string(), contours, width: hi[0] - lo[0], height: hi[1] - lo[1], qty: 1, open_chains })
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

impl NestSettings {
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
    /// 64 cells of row `y` starting at column `x` (outside the map reads empty).
    fn get64(&self, y: usize, x: i64) -> u64 {
        let row = self.row(y);
        let word = |i: i64| -> u64 { if i < 0 { 0 } else { row.get(i as usize).copied().unwrap_or(0) } };
        let w = x.div_euclid(64);
        let s = x.rem_euclid(64) as u32;
        if s == 0 { word(w) } else { (word(w) >> s) | (word(w + 1) << (64 - s)) }
    }
    /// Does `mask` placed with its (0,0) at column `x`, row `y` hit anything set?
    fn hits(&self, mask: &Bits, x: i64, y: i64) -> bool {
        for r in 0..mask.h {
            let sy = y + r as i64;
            if sy < 0 || sy >= self.h as i64 {
                continue;
            }
            let mrow = mask.row(r);
            for (k, m) in mrow.iter().enumerate() {
                if *m != 0 && self.get64(sy as usize, x + 64 * k as i64) & m != 0 {
                    return true;
                }
            }
        }
        false
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
    let grow = ((spacing.max(0.0) / cell).ceil() as usize + 1).min(2000);
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
    Some(Shape { angle, lo, body, halo, grow })
}

/// True-shape nesting on a grid: each part, biggest first, goes where its right edge is nearest the origin (then
/// lowest) at any allowed rotation, so the offcut stays one clean strip across the far end of the sheet, without its spacing halo touching anything placed. Holes in placed parts stay free, so
/// small parts fill them.
pub fn nest(parts: &[Part], s: &NestSettings) -> Nest {
    let mut out = Nest::default();
    let uw = s.sheet_w - 2.0 * s.margin;
    let uh = s.sheet_h - 2.0 * s.margin;
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
    if !(uw > 1.0) || !(uh > 1.0) || !uw.is_finite() || !uh.is_finite() {
        out.unplaced = items;
        return out;
    }
    // About a million cells per sheet, never finer than 0.5 mm.
    let cell = ((uw * uh / 1.0e6).sqrt()).clamp(0.5, 25.0);
    out.cell = cell;
    let gw = (uw / cell).floor() as usize;
    let gh = (uh / cell).floor() as usize;
    let angles = s.angles();
    let shapes: Vec<Vec<Shape>> =
        parts
            .iter()
            .map(|p| {
                if p.qty == 0 { Vec::new() } else { angles.iter().filter_map(|a| raster(p, *a, cell, s.spacing, s.part_in_part, (gw, gh))).collect() }
            })
            .collect();
    items.sort_by(|a, b| {
        let area = |i: usize| parts.get(i).map_or(0.0, |p| p.width * p.height);
        area(*b).total_cmp(&area(*a)).then(a.cmp(b))
    });
    let mut sheets: Vec<Bits> = Vec::new();
    // Holes of placed parts, per sheet, as cell boxes (x0, y0, x1, y1), for part-in-part.
    let mut holes: Vec<Vec<(usize, usize, usize, usize)>> = Vec::new();
    for item in items {
        let Some(variants) = shapes.get(item) else { continue };
        let mut placed = false;
        for sheet in 0..=sheets.len() {
            if sheet == sheets.len() {
                if sheets.len() >= s.max_sheets.max(1) {
                    break;
                }
                sheets.push(Bits::new(gw, gh));
            }
            let Some(grid) = sheets.get(sheet) else { break };
            // Best (top edge, x) over all rotations.
            let mut best: Option<(usize, usize, usize, usize, usize)> = None; // (right edge, y, x, y, variant)
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
                                if !grid.hits(&sh.halo, x as i64 - sh.grow as i64, y as i64 - sh.grow as i64) {
                                    best = Some((0, 0, x, y, vi));
                                    break 'holes;
                                }
                            }
                        }
                    }
                }
            }
            for (vi, sh) in variants.iter().enumerate() {
                if best.is_some_and(|b| b.0 == 0) {
                    break;
                }
                let (w, h) = (sh.body.w, sh.body.h);
                if w > gw || h > gh {
                    continue;
                }
                // Columns from the origin: the first fit in a column is the lowest; the first column with a fit
                // gives this rotation's smallest right edge.
                'cols: for x in 0..=(gw - w) {
                    if best.is_some_and(|b| x + w >= b.0) {
                        break;
                    }
                    for y in 0..=(gh - h) {
                        if !grid.hits(&sh.halo, x as i64 - sh.grow as i64, y as i64 - sh.grow as i64) {
                            let cand = (x + w, y, x, y, vi);
                            if best.is_none_or(|b| (cand.0, cand.1) < (b.0, b.1)) {
                                best = Some((cand.0, cand.1, cand.2, cand.3, cand.4));
                            }
                            break 'cols;
                        }
                    }
                }
            }
            if let Some((_, _, x, y, vi)) = best
                && let Some(sh) = variants.get(vi)
            {
                if let Some(grid) = sheets.get_mut(sheet) {
                    grid.or_at(&sh.body, x, y);
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
        q.cut_in += p.cut_length() / MM_PER_IN * n;
        q.pierces += p.pierces() * p.qty as usize;
        q.minutes += minutes * n;
        q.weight_lb += weight * n;
        part_material += material * n;
        q.machine += machine * n;
        q.consumables += consumables * n;
        let each = (material + machine + consumables) * markup;
        q.lines.push(QuoteLine { name: p.name.clone(), qty: p.qty, each, total: each * n });
    }
    q.material = match (pr.charge_full_sheets, sheets_area_mm2) {
        (true, Some(a)) => lb(a) * pr.per_lb(t.metal),
        _ => part_material,
    };
    q.setup = if parts.iter().any(|p| p.qty > 0) { pr.setup } else { 0.0 };
    let base = q.material + q.machine + q.consumables + q.setup;
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

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PostSettings {
    /// Program in inches (G20) or millimetres (G21): match the QtPlasmaC machine config.
    pub inch: bool,
    pub lead_in: f64,
    /// Select the material in QtPlasmaC with M190 and take its feed rate; otherwise write the tool's feed.
    pub use_material: bool,
    /// Kerf compensation by the controller (G41.1 with the material's kerf).
    pub kerf_comp: bool,
    /// Holes smaller than this diameter run at `hole_speed_pct` (M67 E3); 0 = off.
    pub small_hole: f64,
    pub hole_speed_pct: f64,
}

impl Default for PostSettings {
    fn default() -> Self {
        Self { inch: true, lead_in: 0.2 * MM_PER_IN, use_material: true, kerf_comp: true, small_hole: 1.25 * MM_PER_IN, hole_speed_pct: 60.0 }
    }
}

/// One cut: a placed contour starting at its lead-in.
#[derive(Clone, Debug)]
pub struct Cut {
    pub placement: usize,
    pub hole: bool,
    pub small: bool,
    pub lead_start: P2,
    pub contour: Contour,
}

/// The cuts for one sheet in order: parts kept together (holes first, then the outside), each next part the
/// nearest to the torch, starting from the sheet origin; every contour starts halfway along a segment so the
/// lead-in meets the edge square-on.
pub fn toolpath(parts: &[Part], nest: &Nest, sheet: usize, post: &PostSettings) -> Vec<Cut> {
    let mut left: Vec<usize> = (0..nest.placements.len()).filter(|i| nest.placements.get(*i).is_some_and(|p| p.sheet == sheet)).collect();
    let mut at: P2 = [0.0, 0.0];
    let mut cuts = Vec::new();
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
        let Some(k) = (0..left.len()).min_by(|a, b| key(left.get(*a).unwrap_or(&0)).total_cmp(&key(left.get(*b).unwrap_or(&0)))) else { break };
        let pi = left.swap_remove(k);
        let Some(pl) = nest.placements.get(pi) else { continue };
        let Some(part) = parts.get(pl.part) else { continue };
        let mut holes: Vec<Contour> = part.contours.iter().skip(1).map(|c| pl.contour(c)).collect();
        let outer = part.contours.first().map(|c| pl.contour(c));
        // Holes nearest-first, then the outside.
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
            let (lo, hi) = c.bounds();
            let size = (hi[0] - lo[0]).min(hi[1] - lo[1]);
            // Start halfway along the segment nearest the torch that is long enough for a square lead-in.
            let lead = if hole { post.lead_in.min(size * 0.35) } else { post.lead_in };
            let mut best: Option<(f64, usize)> = None;
            let mut longest = (0.0, 0usize);
            for i in 0..c.segs.len() {
                let l = c.seg_len(i);
                if l > longest.0 {
                    longest = (l, i);
                }
                if l >= (lead * 0.8).max(0.5)
                    && let Some((m, _)) = c.mid_of(i)
                {
                    let d = dist(at, m);
                    if best.is_none_or(|b| d < b.0) {
                        best = Some((d, i));
                    }
                }
            }
            let seg = best.map_or(longest.1, |b| b.1);
            let c = c.restart_at_mid(seg);
            // Direction at the new start: the first segment's tangent there.
            let dir = c.segs.first().map(|s| match *s {
                Seg::Line(p) => {
                    let l = dist(c.start, p).max(1e-12);
                    [(p[0] - c.start[0]) / l, (p[1] - c.start[1]) / l]
                }
                Seg::Arc { c: ctr, ccw, .. } => {
                    let a = (c.start[1] - ctr[1]).atan2(c.start[0] - ctr[0]);
                    if ccw { [-a.sin(), a.cos()] } else { [a.sin(), -a.cos()] }
                }
            });
            let d = dir.unwrap_or([1.0, 0.0]);
            // Scrap side is on the left for both directions (outer clockwise, holes counter-clockwise).
            let lead_start = [c.start[0] - d[1] * lead, c.start[1] + d[0] * lead];
            let small = hole && post.small_hole > 0.0 && (hi[0] - lo[0]).max(hi[1] - lo[1]) < post.small_hole;
            at = c.start;
            cuts.push(Cut { placement: pi, hole, small, lead_start, contour: c });
        }
    }
    cuts
}

/// QtPlasmaC G-code for one sheet.
pub fn gcode(parts: &[Part], nest: &Nest, sheet: usize, tool: &Tool, post: &PostSettings) -> String {
    use std::fmt::Write as _;
    let k = if post.inch { 1.0 / MM_PER_IN } else { 1.0 };
    let n = |v: f64| if post.inch { format!("{:.5}", v * k) } else { format!("{:.4}", v * k) };
    let xy = |p: P2| format!("X{} Y{}", n(p[0]), n(p[1]));
    let mut g = String::new();
    let _ = writeln!(g, "(SolidCraft Plasma CAM - sheet {})", sheet + 1);
    let _ = writeln!(g, "(tool {}: {})", tool.number, tool.name.replace(['(', ')'], ""));
    if post.inch {
        let _ = writeln!(g, "G20 G40 G49 G64 P0.004 G80 G90 G92.1 G94 G97");
    } else {
        let _ = writeln!(g, "G21 G40 G49 G64 P0.1 G80 G90 G92.1 G94 G97");
    }
    if post.use_material {
        let _ = writeln!(g, "M190 P{}", tool.number);
        let _ = writeln!(g, "M66 P3 L3 Q1");
        let _ = writeln!(g, "F#<_hal[plasmac.cut-feed-rate]>");
    } else {
        let _ = writeln!(g, "F{:.1}", tool.feed * k);
    }
    let mut last_part = usize::MAX;
    for cut in toolpath(parts, nest, sheet, post) {
        if cut.placement != last_part {
            last_part = cut.placement;
            let name = nest.placements.get(cut.placement).and_then(|p| parts.get(p.part)).map_or("part", |p| p.name.as_str());
            let _ = writeln!(g, "({})", name.replace(['(', ')'], ""));
        }
        let _ = writeln!(g, "G0 {}", xy(cut.lead_start));
        let _ = writeln!(g, "M3 $0 S1");
        if post.kerf_comp {
            let _ = writeln!(g, "G41.1 D#<_hal[plasmac_run.kerf-width-f]>");
        }
        if cut.small {
            let _ = writeln!(g, "M67 E3 Q{:.0}", post.hole_speed_pct.clamp(10.0, 100.0));
        }
        let _ = writeln!(g, "G1 {}", xy(cut.contour.start));
        let mut at = cut.contour.start;
        for s in &cut.contour.segs {
            match *s {
                Seg::Line(p) => {
                    if dist(at, p) > 1e-6 {
                        let _ = writeln!(g, "G1 {}", xy(p));
                    }
                }
                Seg::Arc { to, c, ccw } => {
                    let _ = writeln!(g, "{} {} I{} J{}", if ccw { "G3" } else { "G2" }, xy(to), n(c[0] - at[0]), n(c[1] - at[1]));
                }
            }
            at = s.end();
        }
        if cut.small {
            let _ = writeln!(g, "M67 E3 Q0");
        }
        if post.kerf_comp {
            let _ = writeln!(g, "G40");
        }
        let _ = writeln!(g, "M5 $0");
    }
    let _ = writeln!(g, "G0 X0 Y0");
    let _ = writeln!(g, "M2");
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
    }

    #[test]
    fn gcode_uses_arcs_material_and_kerf_comp() {
        let parts = sample_parts();
        let n = nest(&parts, &settings(90.0));
        let tool = default_tools().into_iter().find(|t| t.number == 61).unwrap();
        let g = gcode(&parts, &n, 0, &tool, &PostSettings::default());
        let contours: usize = n.placements.iter().filter(|p| p.sheet == 0).map(|p| parts[p.part].contours.len()).sum();
        assert_eq!(g.matches("M3 $0 S1").count(), contours);
        assert_eq!(g.matches("G41.1").count(), contours);
        assert!(g.contains("M190 P61\nM66 P3 L3 Q1\nF#<_hal[plasmac.cut-feed-rate]>"));
        assert!(g.contains("G2 ") && g.contains("G3 "), "circles and corners post as arcs");
        assert!(g.contains("M67 E3 Q60"), "small holes slow down");
        assert!(g.trim_end().ends_with("M2"));
        // Every arc's centre is equidistant from its ends (LinuxCNC rejects a mismatch).
        let mut at = [0.0f64, 0.0];
        for line in g.lines() {
            let val = |c: char| line.split_whitespace().find_map(|w| w.strip_prefix(c)?.parse::<f64>().ok());
            if line.starts_with("G2 ") || line.starts_with("G3 ") {
                let (x, y, i, j) = (val('X').unwrap(), val('Y').unwrap(), val('I').unwrap(), val('J').unwrap());
                let c = [at[0] + i, at[1] + j];
                assert!((dist(at, c) - dist([x, y], c)).abs() < 0.0005, "{line}");
            }
            if let (Some(x), Some(y)) = (val('X'), val('Y')) {
                at = [x, y];
            }
        }
        // A rotated nest still posts true arcs.
        let n45 = nest(&parts, &settings(45.0));
        assert!(gcode(&parts, &n45, 0, &tool, &PostSettings::default()).contains("G2 "));
    }
}
