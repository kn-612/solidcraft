//! Plasma CAM workspace: DXF parts, true-shape sheet nesting, toolpaths with QtPlasmaC G-code (true arcs), a
//! SheetCAM-compatible tool library, and quotes priced by material and thickness. It shares the window, ribbon
//! frame and theme with Design; only the ribbon tabs and the page below them change. The arithmetic lives in
//! `plasma_cam_logic.rs`.

use egui::{Align2, Color32, FontId, Rect, Sense, Shape, Stroke, pos2, vec2};

use crate::SolveApp;
use crate::fonts::RichTextExt;
use crate::theme::Tokens;

#[path = "plasma_cam_logic.rs"]
pub(crate) mod logic;

#[path = "plasma_cam_quote.rs"]
pub(crate) mod quote;

use logic::{Contour, Cut, Lead, MM_PER_IN, Metal, Motion, MoveKind, Nest, NestSettings, Part, PostSettings, Pricing, Side, Tool};

/// `app.ui.tab` while the Plasma CAM workspace is showing.
pub const TAB: &str = "PLASMA";

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Page {
    #[default]
    Setup,
    Nest,
    Toolpath,
    Quote,
}

impl Page {
    const ALL: [Page; 4] = [Page::Setup, Page::Nest, Page::Toolpath, Page::Quote];
    fn label(self) -> &'static str {
        match self {
            Page::Setup => "Setup",
            Page::Nest => "Nest",
            Page::Toolpath => "Toolpath",
            Page::Quote => "Quote",
        }
    }
}

/// Which tools the library list shows.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Filter {
    All,
    Metal(Metal),
    Oxy,
}

impl Filter {
    const ALL: [Filter; 5] = [Filter::All, Filter::Metal(Metal::Steel), Filter::Metal(Metal::Stainless), Filter::Metal(Metal::Aluminum), Filter::Oxy];
    fn label(self) -> &'static str {
        match self {
            Filter::All => "All",
            Filter::Metal(m) => m.label(),
            Filter::Oxy => "Oxy-fuel",
        }
    }
    fn keeps(self, t: &Tool) -> bool {
        match self {
            Filter::All => true,
            Filter::Metal(m) => !t.oxy && t.metal == m,
            Filter::Oxy => t.oxy,
        }
    }
}

const ROTATIONS: [(f64, &str); 6] = [
    (0.0, "No Rotation"),
    (180.0, "Rotate 180°"),
    (90.0, "Rotate 90°"),
    (45.0, "Rotate 45°"),
    (15.0, "Rotate 15°"),
    (logic::BEST_FIT, "Any (best fit)"),
];

/// A part being moved by hand.
#[derive(Clone, Copy, Debug)]
struct Drag {
    placement: usize,
    /// Pointer at the grab, sheet mm.
    grab: [f64; 2],
    /// The placement's offset at the grab.
    from: (f64, f64),
}

/// Sheet mm ↔ screen for the sheet view's top-down camera.
#[derive(Clone, Copy, Debug)]
struct SheetView {
    scale: f64,
    ox: f64,
    oy: f64,
}

impl SheetView {
    fn of(cam: &solidcraft_engine::render::Camera, rect: Rect) -> Self {
        let scale = f64::from(rect.height()) / (2.0 * cam.half_height()).max(1e-9);
        SheetView { scale, ox: f64::from(rect.center().x) - cam.target.x * scale, oy: f64::from(rect.center().y) + cam.target.y * scale }
    }
    fn map(&self, q: [f64; 2]) -> egui::Pos2 {
        pos2((self.ox + q[0] * self.scale) as f32, (self.oy - q[1] * self.scale) as f32)
    }
    fn sheet_at(&self, p: egui::Pos2) -> [f64; 2] {
        [(f64::from(p.x) - self.ox) / self.scale, (self.oy - f64::from(p.y)) / self.scale]
    }
}

fn sheet_camera() -> solidcraft_engine::render::Camera {
    let mut cam = solidcraft_engine::render::Camera::default();
    cam.set_view(solidcraft_engine::render::StandardView::Top);
    cam.fov = 0.0;
    cam.up = solidcraft_engine::render::UpAxis::Z;
    cam
}

/// The toolpath and G-code for one sheet, kept until something they depend on changes.
struct Posted {
    key: String,
    cuts: Vec<Cut>,
    code: String,
    motions: Vec<Motion>,
    /// Program run time, seconds.
    total: f64,
}

/// The torch simulation on the Toolpath tab.
#[derive(Clone, Copy, Debug)]
struct Sim {
    /// Program time shown, seconds.
    t: f64,
    playing: bool,
    /// Program seconds per real second.
    speed: f64,
}

const SIM_SPEEDS: [f64; 6] = [1.0, 2.0, 5.0, 10.0, 25.0, 100.0];

pub struct State {
    pub page: Page,
    pub parts: Vec<Part>,
    /// Colour slot per part (stable when parts are removed).
    colours: Vec<usize>,
    next_colour: usize,
    /// Shading triangles per part, per contour.
    tris: Vec<Vec<Vec<[usize; 3]>>>,
    pub selected: Option<usize>,
    pub sheet_w_in: f64,
    pub sheet_h_in: f64,
    pub spacing_in: f64,
    pub margin_in: f64,
    pub rotation_step: f64,
    pub part_in_part: bool,
    pub max_sheets: usize,
    pub nest: Option<Nest>,
    /// Bumped whenever the nest changes, so cached toolpaths know.
    nest_rev: u64,
    #[cfg(not(target_arch = "wasm32"))]
    job: Option<std::sync::mpsc::Receiver<Nest>>,
    pub sheet: usize,
    pub tools: Vec<Tool>,
    pub tool: usize,
    filter: Filter,
    /// Scroll the tool list to the picked tool on the next frame.
    reveal_tool: bool,
    pub pricing: Pricing,
    pub post: PostSettings,
    posted: Option<Posted>,
    sim: Option<Sim>,
    pub metric: bool,
    pub message: Option<(String, bool)>,
    /// The sheet view's camera (zoom and pan), framed on the sheet until the user zooms.
    cam: solidcraft_engine::render::Camera,
    fitted: bool,
    view_key: Option<(u64, u64)>,
    drag: Option<Drag>,
    /// The last moved part, what it touches, and whether it left the sheet.
    clash: Option<(usize, Vec<usize>, bool)>,
    pub business: quote::Business,
    pub customer: quote::Customer,
    /// The saved business info and prices, to notice edits.
    saved_json: String,
}

impl Default for State {
    fn default() -> Self {
        let tools = logic::default_tools();
        let tool = tools.iter().position(|t| t.name == "65A 1/4 Steel").unwrap_or(0);
        Self {
            page: Page::Setup,
            parts: Vec::new(),
            colours: Vec::new(),
            next_colour: 0,
            tris: Vec::new(),
            selected: None,
            sheet_w_in: 96.0,
            sheet_h_in: 48.0,
            spacing_in: 0.25,
            margin_in: 0.5,
            rotation_step: 90.0,
            part_in_part: true,
            max_sheets: 50,
            nest: None,
            nest_rev: 0,
            #[cfg(not(target_arch = "wasm32"))]
            job: None,
            sheet: 0,
            tools,
            tool,
            filter: Filter::Metal(Metal::Steel),
            reveal_tool: true,
            pricing: Pricing::default(),
            post: PostSettings::default(),
            posted: None,
            sim: None,
            metric: false,
            message: None,
            cam: sheet_camera(),
            fitted: false,
            view_key: None,
            drag: None,
            clash: None,
            business: quote::Business::default(),
            customer: quote::Customer::default(),
            saved_json: String::new(),
        }
        .with_saved()
    }
}

impl State {
    /// Load the business info and shop prices kept from earlier sessions.
    fn with_saved(mut self) -> Self {
        let saved = quote::load();
        self.business = saved.business.clone();
        if let Some(p) = &saved.pricing {
            p.apply(&mut self.pricing);
        }
        self.saved_json = self.saved_snapshot();
        self
    }
    fn saved_snapshot(&self) -> String {
        let saved = quote::Saved { business: self.business.clone(), pricing: Some(quote::SavedPricing::from(&self.pricing)) };
        serde_json::to_string(&saved).unwrap_or_default()
    }
    /// Write the business info and prices when they changed.
    fn autosave(&mut self) {
        let now = self.saved_snapshot();
        if now != self.saved_json {
            let saved = quote::Saved { business: self.business.clone(), pricing: Some(quote::SavedPricing::from(&self.pricing)) };
            if let Err(e) = quote::save(&saved) {
                self.message = Some((format!("Could not save business info: {e}"), true));
            }
            self.saved_json = now;
        }
    }
    /// Frame the whole sheet in `rect`.
    fn fit(&mut self, rect: Rect) {
        let (w, h) = (self.sheet_w_in.max(0.1) * MM_PER_IN, self.sheet_h_in.max(0.1) * MM_PER_IN);
        let area = rect.shrink(24.0);
        let scale = (f64::from(area.width().max(10.0)) / w).min(f64::from(area.height().max(10.0)) / h);
        self.cam.target.x = w * 0.5;
        self.cam.target.y = h * 0.5;
        self.cam.target.z = 0.0;
        self.cam.distance = f64::from(rect.height()) / (2.0 * scale) / 0.4;
        self.fitted = true;
    }
    fn settings(&self) -> NestSettings {
        NestSettings {
            sheet_w: self.sheet_w_in * MM_PER_IN,
            sheet_h: self.sheet_h_in * MM_PER_IN,
            spacing: self.spacing_in * MM_PER_IN,
            margin: self.margin_in * MM_PER_IN,
            rotation_step: self.rotation_step,
            max_sheets: self.max_sheets,
            part_in_part: self.part_in_part,
        }
    }
    fn tool(&self) -> Option<&Tool> {
        self.tools.get(self.tool).or_else(|| self.tools.first())
    }
    fn colour(&self, part: usize) -> Color32 {
        crate::theme::part_colour(self.colours.get(part).copied().unwrap_or(part))
    }
    fn add_part(&mut self, part: Part) {
        self.tris.push(part.contours.iter().map(|c| logic::triangulate(&c.pts)).collect());
        self.parts.push(part);
        self.colours.push(self.next_colour);
        self.next_colour += 1;
        self.set_nest(None);
    }
    fn set_nest(&mut self, nest: Option<Nest>) {
        self.nest = nest;
        self.nest_rev += 1;
        self.posted = None;
        self.sim = None;
        self.drag = None;
        self.clash = None;
        #[cfg(not(target_arch = "wasm32"))]
        {
            self.job = None;
        }
    }
    fn clear(&mut self) {
        self.parts.clear();
        self.tris.clear();
        self.colours.clear();
        self.next_colour = 0;
        self.selected = None;
        self.sheet = 0;
        self.set_nest(None);
    }
    fn remove(&mut self, i: usize) {
        if i < self.parts.len() {
            self.parts.remove(i);
        }
        if i < self.tris.len() {
            self.tris.remove(i);
        }
        if i < self.colours.len() {
            self.colours.remove(i);
        }
        self.selected = None;
        self.set_nest(None);
    }
    fn busy(&self) -> bool {
        #[cfg(not(target_arch = "wasm32"))]
        {
            self.job.is_some()
        }
        #[cfg(target_arch = "wasm32")]
        {
            false
        }
    }
    fn run_nest(&mut self) {
        if self.parts.is_empty() {
            self.message = Some(("Add parts before nesting.".into(), true));
            return;
        }
        let parts = self.parts.clone();
        let settings = self.settings();
        self.set_nest(None);
        self.sheet = 0;
        #[cfg(not(target_arch = "wasm32"))]
        {
            let (tx, rx) = std::sync::mpsc::channel();
            let spawned = std::thread::Builder::new().name("plasma-nest".into()).spawn(move || {
                let _ = tx.send(logic::nest(&parts, &settings));
            });
            match spawned {
                Ok(_) => {
                    self.job = Some(rx);
                    let text = if settings.rotation_step == logic::BEST_FIT {
                        format!("Searching for the best fit (up to {} s)…", logic::SEARCH_BUDGET.as_secs())
                    } else {
                        "Nesting…".into()
                    };
                    self.message = Some((text, false));
                }
                Err(e) => self.message = Some((format!("Could not start nesting: {e}"), true)),
            }
        }
        #[cfg(target_arch = "wasm32")]
        {
            let n = logic::nest(&parts, &settings);
            self.finish_nest(n);
        }
    }
    fn finish_nest(&mut self, n: Nest) {
        let placed = n.placements.len();
        self.message = Some(if n.unplaced.is_empty() && n.truncated == 0 {
            (format!("Nested {placed} parts on {} sheet(s).", n.sheets), false)
        } else {
            (format!("Nested {placed} parts; {} did not fit the sheet.", n.unplaced.len() + n.truncated), true)
        });
        self.set_nest(Some(n));
    }
    /// Collect a finished background nest.
    fn poll(&mut self, ctx: &egui::Context) {
        #[cfg(not(target_arch = "wasm32"))]
        if let Some(rx) = &self.job {
            match rx.try_recv() {
                Ok(n) => {
                    self.job = None;
                    self.finish_nest(n);
                }
                Err(std::sync::mpsc::TryRecvError::Empty) => ctx.request_repaint_after(std::time::Duration::from_millis(50)),
                Err(std::sync::mpsc::TryRecvError::Disconnected) => {
                    self.job = None;
                    self.message = Some(("Nesting stopped unexpectedly.".into(), true));
                }
            }
        }
        #[cfg(target_arch = "wasm32")]
        let _ = ctx;
    }
    /// The toolpath and G-code for the current sheet (cached).
    fn posted(&mut self) -> Option<&Posted> {
        let n = self.nest.as_ref()?;
        let tool = self.tool()?.clone();
        let sides: Vec<_> = self.parts.iter().map(|p| (p.side, p.reverse, p.flip_lead)).collect();
        let key = format!("{}|{}|{:?}|{:?}|{:?}", self.nest_rev, self.sheet, tool, self.post, sides);
        if self.posted.as_ref().is_none_or(|p| p.key != key) {
            let cuts = logic::toolpath(&self.parts, n, self.sheet, &self.post, tool.kerf);
            let code = logic::gcode(&self.parts, n, self.sheet, &tool, &self.post);
            let motions = logic::motions(&cuts, &tool, &self.post);
            let total = motions.iter().map(|m| m.seconds).sum();
            self.posted = Some(Posted { key, cuts, code, motions, total });
        }
        self.posted.as_ref()
    }
    /// The parts a cut-side button acts on: the selected one, else all.
    fn targets(&self) -> Vec<usize> {
        match self.selected {
            Some(i) if i < self.parts.len() => vec![i],
            _ => (0..self.parts.len()).collect(),
        }
    }
    fn target_name(&self) -> String {
        match self.selected.and_then(|i| self.parts.get(i)) {
            Some(p) => p.name.clone(),
            None => "all parts".into(),
        }
    }
    /// Length in the display unit, with its suffix.
    fn len(&self, mm: f64) -> String {
        if self.metric { format!("{mm:.1} mm") } else { format!("{:.2} in", mm / MM_PER_IN) }
    }
    #[allow(dead_code)]
    fn feed(&self, mm_min: f64) -> String {
        if self.metric { format!("{mm_min:.0} mm/min") } else { format!("{:.0} ipm", mm_min / MM_PER_IN) }
    }
}

pub fn active(app: &SolveApp) -> bool {
    app.ui.tab == TAB
}

/// Switch to the Plasma CAM workspace. Leaving it is entering Design or Render.
pub fn enter(app: &mut SolveApp) -> Result<(), String> {
    if app.session.active_sketch.is_some() || app.tool.is_some() || app.dialog.is_some() || app.file_load.busy() {
        return Err(crate::i18n::tr("render.error_finish_operation").into());
    }
    if crate::render_workspace::active(app) {
        crate::render_workspace::enter(app, false)?;
    }
    app.ui.tab = TAB.into();
    app.viewport.hover = None;
    app.cam_anim = None;
    Ok(())
}

// --- Ribbon --------------------------------------------------------------------------------------

struct Button {
    id: &'static str,
    label: String,
    icon: &'static str,
    on: bool,
    /// Shown pressed (the current choice).
    active: bool,
}

fn button(id: &'static str, label: &str, icon: &'static str) -> Button {
    Button { id, label: label.to_string(), icon, on: true, active: false }
}

fn groups(app: &SolveApp) -> Vec<(&'static str, Vec<Button>)> {
    let s = &app.plasma_cam;
    let has_parts = !s.parts.is_empty() && !s.busy();
    let nested = s.nest.is_some();
    let rotation = ROTATIONS.iter().find(|(a, _)| (*a - s.rotation_step).abs() < 0.1).map_or("Rotate 90°", |r| r.1);
    let mut out = vec![(
        "Parts",
        vec![
            button("import", "Import DXF", "dxf"),
            button("samples", "Sample Parts", "library"),
            Button { on: has_parts, ..button("clear", "Clear Parts", "delete") },
        ],
    )];
    let sheets =
        vec![Button { on: nested, ..button("prev", "Previous", "sheet_prev") }, Button { on: nested, ..button("next", "Next", "sheet_next") }];
    match s.page {
        Page::Setup => {
            out.push(("Tools", vec![button("import_tools", "Import Tools", "import"), button("export_materials", "QtPlasmaC Materials", "export")]));
            out.push(("Units", vec![button("units", if s.metric { "Inches" } else { "Millimetres" }, "dimension")]));
            out.push(("Nest", vec![Button { on: has_parts, ..button("nest", "Nest Parts", "pattern_rect") }]));
        }
        Page::Nest => {
            out.push((
                "Nest",
                vec![
                    Button { on: has_parts, ..button("nest", "Nest Parts", "pattern_rect") },
                    button("rotate", rotation, "pattern_circ"),
                    Button { active: s.part_in_part, ..button("pip", "Small Parts in Holes", "part_in_part") },
                    Button { on: nested, ..button("unnest", "Clear Nest", "revert") },
                ],
            ));
            out.push(("Sheets", sheets));
        }
        Page::Toolpath => {
            let simulating = s.sim.is_some();
            out.push((
                "Simulate",
                vec![Button {
                    on: nested,
                    active: simulating,
                    ..button(
                        "simulate",
                        if simulating { "Stop Simulation" } else { "Simulate" },
                        if simulating { "timeline_stop" } else { "timeline_play" },
                    )
                }],
            ));
            out.push((
                "Post",
                vec![
                    Button { on: nested, ..button("post", "Post G-code", "export") },
                    Button { on: nested, ..button("copy_gcode", "Copy G-code", "copy") },
                ],
            ));
            out.push(("Sheets", sheets));
        }
        Page::Quote => {
            out.push((
                "Customer quote",
                vec![
                    Button { on: has_parts, ..button("print_quote", "Print Quote", "print_quote") },
                    Button { on: has_parts, ..button("save_quote", "Save Quote", "export") },
                ],
            ));
            out.push((
                "Quote",
                vec![
                    Button { on: has_parts, ..button("copy_quote", "Copy Quote", "copy") },
                    Button { on: has_parts, ..button("nest", "Nest for Sheets", "pattern_rect") },
                ],
            ));
        }
    }
    out
}

/// The ribbon card's contents: this workspace's tabs, then its command groups.
pub fn ribbon(app: &mut SolveApp, ui: &mut egui::Ui, r: Rect) {
    let t = Tokens::get();
    let accent = crate::theme::TabAccent::for_tab(&t, "MODEL");
    let inset = 4.0;
    let row_h = crate::toolbar::TAB_ROW_HEIGHT;
    let mut x = r.left() + inset;
    for page in Page::ALL {
        let label = crate::i18n::tr_label(page.label());
        let w = ui.painter().layout_no_wrap(label.to_string(), crate::fonts::semibold(t.font_button), t.text).size().x + 24.0;
        let tr = Rect::from_min_size(pos2(x, r.top() + inset), vec2(w, row_h - 2.0 * inset));
        let resp = ui.interact(tr, ui.id().with(("plasma_tab", page.label())), Sense::click());
        let active = app.plasma_cam.page == page;
        let (fill, ink) = if active {
            (accent.soft, accent.ink)
        } else if resp.hovered() {
            (t.hover, t.text)
        } else {
            (Color32::TRANSPARENT, t.text)
        };
        ui.painter().rect_filled(tr, t.control_radius.saturating_sub(1), fill);
        ui.painter().text(
            tr.center(),
            Align2::CENTER_CENTER,
            label,
            if active { crate::fonts::semibold(t.font_button) } else { FontId::proportional(t.font_button) },
            ink,
        );
        crate::scenario::publish_target(&format!("plasma:tab:{}", page.label().to_lowercase()), tr);
        if resp.clicked() {
            app.plasma_cam.page = page;
        }
        x += w + 2.0;
    }
    let top = r.top() + row_h;
    let bottom = top + crate::toolbar::GROUP_HEIGHT - 2.0;
    let mut x = r.left() + inset;
    let mut clicked = None;
    for (gi, (caption, buttons)) in groups(app).into_iter().enumerate() {
        if gi > 0 {
            ui.painter().line_segment([pos2(x, top + 8.0), pos2(x, bottom - 6.0)], Stroke::new(1.0, t.border));
            x += 4.0;
        }
        let start = x;
        for b in &buttons {
            let label = crate::i18n::tr_label(&b.label);
            let w = (ui.painter().layout_no_wrap(label.to_string(), FontId::proportional(t.font_button), t.text).size().x + 14.0).clamp(58.0, 120.0);
            let br = Rect::from_min_size(pos2(x, top + 3.0), vec2(w, 60.0));
            let resp = ui.interact(br, ui.id().with(("plasma_button", b.id)), if b.on { Sense::click() } else { Sense::hover() });
            if b.active {
                ui.painter().rect_filled(br, t.control_radius, accent.soft);
            } else if b.on && resp.hovered() {
                ui.painter().rect_filled(br, t.control_radius, t.hover);
            }
            crate::icons::paint_icon(
                ui.painter(),
                Rect::from_center_size(pos2(br.center().x, br.top() + 16.0), vec2(28.0, 28.0)),
                b.icon,
                if b.on { 1.0 } else { 0.35 },
            );
            ui.painter().text(
                pos2(br.center().x, br.top() + 40.0),
                Align2::CENTER_CENTER,
                label,
                FontId::proportional(t.font_button),
                if b.on { t.text } else { t.text_dim },
            );
            crate::scenario::publish_target(&format!("plasma:{}", b.id), br);
            if b.on && resp.clicked() {
                clicked = Some(b.id);
            }
            x += w + 3.0;
        }
        ui.painter().text(
            pos2((start + x) / 2.0, bottom - 10.0),
            Align2::CENTER_CENTER,
            crate::i18n::tr_label(caption),
            FontId::proportional(t.font_small),
            t.text_dim,
        );
        x += 4.0;
    }
    if let Some(id) = clicked {
        act(app, ui.ctx(), id);
    }
}

fn act(app: &mut SolveApp, ctx: &egui::Context, id: &str) {
    let s = &mut app.plasma_cam;
    match id {
        "samples" => {
            for p in logic::sample_parts() {
                s.add_part(p);
            }
            s.message = Some(("Added six sample parts.".into(), false));
        }
        "clear" => s.clear(),
        "nest" => {
            s.run_nest();
            if s.page == Page::Setup {
                s.page = Page::Nest;
            }
        }
        "unnest" => s.set_nest(None),
        "rotate" => {
            let i = ROTATIONS.iter().position(|(a, _)| (*a - s.rotation_step).abs() < 0.1).unwrap_or(2);
            s.rotation_step = ROTATIONS.get((i + 1) % ROTATIONS.len()).map_or(90.0, |r| r.0);
            s.set_nest(None);
        }
        "pip" => {
            s.part_in_part = !s.part_in_part;
            s.set_nest(None);
        }
        "units" => s.metric = !s.metric,
        "side_out" | "side_in" | "side_on" => {
            let side = match id {
                "side_in" => Side::Inside,
                "side_on" => Side::OnLine,
                _ => Side::Outside,
            };
            for i in s.targets() {
                if let Some(p) = s.parts.get_mut(i) {
                    p.side = side;
                }
            }
            s.message = Some((format!("{} offset: {}", side.label(), s.target_name()), false));
        }
        "reverse" => {
            for i in s.targets() {
                if let Some(p) = s.parts.get_mut(i) {
                    p.reverse = !p.reverse;
                }
            }
            s.message = Some((format!("Cut direction reversed: {}", s.target_name()), false));
        }
        "flip_lead" => {
            for i in s.targets() {
                if let Some(p) = s.parts.get_mut(i).filter(|p| p.side == Side::OnLine) {
                    p.flip_lead = !p.flip_lead;
                }
            }
            s.message = Some((format!("Leads moved to the other side: {}", s.target_name()), false));
        }
        "simulate" => {
            s.sim = match s.sim {
                Some(_) => None,
                None => Some(Sim { t: 0.0, playing: true, speed: 10.0 }),
            };
        }
        "prev" => s.sheet = s.sheet.saturating_sub(1),
        "next" => {
            let n = s.nest.as_ref().map_or(1, |n| n.sheets.max(1));
            s.sheet = (s.sheet + 1).min(n - 1);
        }
        "copy_quote" => {
            ctx.copy_text(quote_text(s));
            s.message = Some(("Quote copied to the clipboard.".into(), false));
        }
        "copy_gcode" => {
            let sheet = s.sheet;
            if let Some(code) = s.posted().map(|p| p.code.clone()) {
                ctx.copy_text(code);
                s.message = Some((format!("G-code for sheet {} copied.", sheet + 1), false));
            }
        }
        "post" => post(app),
        "print_quote" => print_quote(app, ctx),
        "save_quote" => save_quote(app),
        "import" => import(app),
        "import_tools" => import_tools(app),
        "export_materials" => export_materials(app),
        _ => {}
    }
}

fn file_name(path: &str) -> String {
    path.rsplit(['/', '\\']).next().unwrap_or(path).to_string()
}

fn import(app: &mut SolveApp) {
    app.pick_open_for(&["dxf"], |app, path| {
        if let Some(path) = path {
            import_path(app, &path);
        }
    });
}

/// Add the DXF at `path` to the parts list (Import DXF, a dropped file, the control channel's `ui.file`).
pub fn import_path(app: &mut SolveApp, path: &str) {
    let name = file_name(path);
    let result = solidcraft_engine::io::vfs::read_to_string(path, solidcraft_engine::io::MAX_DRAWING_BYTES)
        .map_err(|e| format!("{name}: {e}"))
        .and_then(|text| logic::part_from_dxf(&name, &text));
    let s = &mut app.plasma_cam;
    match result {
        Ok(part) => {
            s.message = Some(if part.open_chains > 0 {
                (format!("{name}: {} open outline(s) skipped; close them in your CAD.", part.open_chains), true)
            } else {
                (format!("Imported {name}."), false)
            });
            s.add_part(part);
            s.selected = Some(s.parts.len() - 1);
        }
        Err(e) => s.message = Some((e, true)),
    }
}

/// Is this a drawing Plasma Nest takes as a part (while the workspace is open)?
pub fn takes_path(app: &SolveApp, path: &str) -> bool {
    app.ui.tab == "PLASMA" && takes_path_str(path)
}

/// DXF files dropped on the window while Plasma Nest is open become parts (any number at once);
/// other files are left for the app to open as usual.
pub fn take_drops(app: &mut SolveApp, ctx: &egui::Context) {
    if app.ui.tab != "PLASMA" {
        return;
    }
    let dropped = ctx.input(|i| i.raw.dropped_files.clone());
    let paths: Vec<String> = dropped.iter().map(|f| f.path().to_string_lossy().to_string()).filter(|p| takes_path_str(p)).collect();
    if paths.is_empty() {
        return;
    }
    ctx.input_mut(|i| i.raw.dropped_files.retain(|f| !takes_path_str(&f.path().to_string_lossy())));
    for p in &paths {
        import_path(app, p);
    }
    if paths.len() > 1 {
        app.plasma_cam.message = Some((format!("Imported {} drawings.", paths.len()), false));
    }
}

fn takes_path_str(path: &str) -> bool {
    path.to_ascii_lowercase().ends_with(".dxf")
}

fn import_tools(app: &mut SolveApp) {
    app.pick_open_for(&["tools"], |app, path| {
        let Some(path) = path else { return };
        let name = file_name(&path);
        let s = &mut app.plasma_cam;
        match solidcraft_engine::io::vfs::read_to_string(&path, 16 << 20) {
            Ok(text) => {
                let tools = logic::parse_tools(&text);
                if tools.is_empty() {
                    s.message = Some((format!("{name}: no plasma tools found."), true));
                } else {
                    s.message = Some((format!("Loaded {} tools from {name}.", tools.len()), false));
                    s.tools = tools;
                    s.tool = 0;
                    s.posted = None;
                }
            }
            Err(e) => s.message = Some((format!("{name}: {e}"), true)),
        }
    });
}

fn export_materials(app: &mut SolveApp) {
    let text = logic::qtplasmac_materials(&app.plasma_cam.tools, app.plasma_cam.post.inch);
    app.pick_save("machine_material.cfg", &["cfg"], move |app, path| {
        let Some(path) = path else { return };
        app.plasma_cam.message = Some(match solidcraft_engine::io::vfs::write(&path, text.as_bytes()) {
            Ok(()) => (format!("Saved {path}"), false),
            Err(e) => (format!("Could not save materials: {e}"), true),
        });
    });
}

fn post(app: &mut SolveApp) {
    let sheet = app.plasma_cam.sheet;
    let Some(text) = app.plasma_cam.posted().map(|p| p.code.clone()) else { return };
    let name = format!("{}-sheet{}.ngc", app.session.doc.name, sheet + 1);
    app.pick_save(&name, &["ngc", "nc", "tap"], move |app, path| {
        let Some(path) = path else { return };
        app.plasma_cam.message = Some(match solidcraft_engine::io::vfs::write(&path, text.as_bytes()) {
            Ok(()) => (format!("Saved {path}"), false),
            Err(e) => (format!("Could not save G-code: {e}"), true),
        });
    });
}

fn sheets_area(s: &State) -> Option<f64> {
    s.nest.as_ref().map(|n| n.sheets as f64 * s.settings().sheet_w * s.settings().sheet_h)
}

fn quote_text(s: &State) -> String {
    use std::fmt::Write as _;
    let Some(tool) = s.tool() else { return String::new() };
    let q = logic::quote(&s.parts, tool, &s.pricing, sheets_area(s));
    let mut out = format!("Plasma cutting quote - {} ({})\n", tool.name, tool.metal.label());
    for l in &q.lines {
        let _ = writeln!(out, "{:<28} {:>4} x ${:>9.2} = ${:>10.2}", l.name, l.qty, l.each, l.total);
    }
    let _ = writeln!(out, "Total: ${:.2}", q.total);
    out
}

// --- Page ----------------------------------------------------------------------------------------

/// The workspace page, in place of the 3D viewport.
pub fn show(app: &mut SolveApp, ui: &mut egui::Ui) {
    let t = Tokens::get();
    app.plasma_cam.poll(ui.ctx());
    let mut full = ui.max_rect();
    // Floating layout draws the page under the cards; start below them.
    let cards_bottom = app.shell.cards.iter().map(|c| c.bottom()).fold(full.top(), f32::max);
    if cards_bottom < full.bottom() - 100.0 {
        full.min.y = cards_bottom;
    }
    ui.painter().rect_filled(ui.max_rect(), 0, t.viewport_bottom);
    let gap = t.space_8;
    let area = full.shrink(gap);
    let left = Rect::from_min_size(area.min, vec2(300.0_f32.min(area.width() * 0.3), area.height()));
    let right_w = 360.0_f32.min(area.width() * 0.3);
    let right = Rect::from_min_max(pos2(area.right() - right_w, area.top()), area.max);
    let centre = Rect::from_min_max(pos2(left.right() + gap, area.top()), pos2(right.left() - gap, area.bottom()));
    card(ui, left);
    card(ui, right);
    parts_panel(app, ui, left.shrink(t.space_12));
    match app.plasma_cam.page {
        Page::Setup => tools_panel(app, ui, right.shrink(t.space_12)),
        Page::Nest => nest_panel(app, ui, right.shrink(t.space_12)),
        Page::Toolpath => toolpath_panel(app, ui, right.shrink(t.space_12)),
        Page::Quote => quote_panel(app, ui, right.shrink(t.space_12)),
    }
    sheet_canvas(app, ui, centre);
}

/// One line of text, cut short with "…" past `width`.
fn one_line(ui: &egui::Ui, text: String, font: FontId, colour: Color32, width: f32) -> std::sync::Arc<egui::Galley> {
    let mut job = egui::text::LayoutJob::simple_singleline(text, font, colour);
    job.wrap.max_width = width;
    job.wrap.max_rows = 1;
    job.wrap.break_anywhere = true;
    ui.painter().layout_job(job)
}

fn card(ui: &egui::Ui, r: Rect) {
    let t = Tokens::get();
    ui.painter().rect(r, t.radius_lg, t.panel, Stroke::new(1.0, t.border), egui::StrokeKind::Inside);
}

fn child(ui: &mut egui::Ui, r: Rect, salt: &str) -> egui::Ui {
    let mut c = ui.new_child(egui::UiBuilder::new().max_rect(r).id_salt(salt).layout(egui::Layout::top_down(egui::Align::Min)));
    c.set_clip_rect(r.expand(2.0));
    c
}

fn heading(ui: &mut egui::Ui, text: &str, detail: &str) {
    let t = Tokens::get();
    ui.horizontal(|ui| {
        ui.label(egui::RichText::new(crate::i18n::tr_label(text).to_uppercase()).size(t.font_small).semibold().color(t.text_dim));
        if !detail.is_empty() {
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                ui.label(egui::RichText::new(detail).size(t.font_small).color(t.text_dim));
            });
        }
    });
    ui.add_space(t.space_4);
}

fn note(ui: &mut egui::Ui, text: &str) {
    let t = Tokens::get();
    ui.label(egui::RichText::new(crate::i18n::tr_label(text)).size(t.font_small).color(t.text_dim));
}

fn stat(ui: &mut egui::Ui, label: &str, value: String) {
    let t = Tokens::get();
    ui.horizontal(|ui| {
        ui.label(egui::RichText::new(crate::i18n::tr_label(label)).size(t.font_body).color(t.text_dim));
        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
            ui.label(egui::RichText::new(value).size(t.font_body).color(t.text));
        });
    });
}

fn num(ui: &mut egui::Ui, label: &str, value: &mut f64, range: std::ops::RangeInclusive<f64>, speed: f64, suffix: &str) -> bool {
    crate::ui_kit::row(ui, label, |ui| crate::ui_kit::number_field(ui, value, range, speed, suffix).changed())
}

/// A length field shown in the display unit and stored in `k`-scaled storage units (`k` = display per stored).
fn scaled(ui: &mut egui::Ui, label: &str, value: &mut f64, k: f64, max: f64, speed: f64, suffix: &str) -> bool {
    let mut d = *value * k;
    if num(ui, label, &mut d, 0.0..=max * k, speed * k, suffix) {
        *value = d / k;
        return true;
    }
    false
}

fn parts_panel(app: &mut SolveApp, ui: &mut egui::Ui, r: Rect) {
    let t = Tokens::get();
    let mut ui = child(ui, r, "plasma_parts");
    let s = &mut app.plasma_cam;
    let total: u32 = s.parts.iter().map(|p| p.qty).sum();
    heading(&mut ui, "Parts", &format!("{} files · {total} pcs", s.parts.len()));
    let list_h = (r.height() - 290.0).max(120.0);
    let mut remove = None;
    let mut changed = false;
    egui::ScrollArea::vertical().id_salt("plasma_part_rows").max_height(list_h).auto_shrink([false, false]).show(&mut ui, |ui| {
        if s.parts.is_empty() {
            ui.add_space(t.space_24);
            ui.vertical_centered(|ui| {
                ui.label(egui::RichText::new(crate::i18n::tr_label("No parts yet")).size(t.font_heading).color(t.text));
                ui.label(egui::RichText::new(crate::i18n::tr_label("Import DXF files or add the sample parts.")).size(t.font_body).color(t.text_dim));
            });
        }
        for i in 0..s.parts.len() {
            let (rr, resp) = ui.allocate_exact_size(vec2(ui.available_width(), 58.0), Sense::click());
            let sel = s.selected == Some(i);
            if sel {
                ui.painter().rect_filled(rr, t.control_radius, t.accent_soft);
            } else if resp.hovered() {
                ui.painter().rect_filled(rr, t.control_radius, t.hover);
            }
            if resp.clicked() {
                s.selected = if sel { None } else { Some(i) };
            }
            let colour = s.colour(i);
            let thumb = Rect::from_min_size(rr.min + vec2(4.0, 7.0), vec2(44.0, 44.0));
            ui.painter().rect_filled(thumb, t.radius_md, t.panel_header);
            let metric = s.metric;
            if let (Some(p), Some(tr)) = (s.parts.get(i), s.tris.get(i)) {
                let k = (thumb.width() - 8.0) as f64 / p.width.max(p.height).max(1e-6);
                let off = thumb.center() - vec2((p.width * k) as f32 / 2.0, -(p.height * k) as f32 / 2.0);
                let map = |q: [f64; 2]| pos2(off.x + (q[0] * k) as f32, off.y - (q[1] * k) as f32);
                paint_part(ui.painter(), &p.contours, tr, &map, colour, t.panel_header, 1.0);
            }
            let Some(p) = s.parts.get_mut(i) else { continue };
            let text_w = (rr.width() - 176.0).max(20.0);
            let name = one_line(ui, p.name.clone(), FontId::proportional(t.font_body), t.text, text_w);
            ui.painter().galley(pos2(thumb.right() + 8.0, rr.top() + 5.0), name, t.text);
            let size = if metric {
                format!("{:.1} × {:.1} mm", p.width, p.height)
            } else {
                format!("{:.2} × {:.2} in", p.width / MM_PER_IN, p.height / MM_PER_IN)
            };
            let size = one_line(ui, size, FontId::proportional(t.font_small), t.text_dim, text_w);
            ui.painter().galley(pos2(thumb.right() + 8.0, rr.top() + 23.0), size, t.text_dim);
            let side = if p.reverse { format!("{} · reversed", p.side.label()) } else { p.side.label().to_string() };
            let side = one_line(ui, side, FontId::proportional(t.font_small), t.text_dim, text_w);
            ui.painter().galley(pos2(thumb.right() + 8.0, rr.top() + 38.0), side, t.text_dim);
            // Quantity: − and + step it; click the number to type one.
            let y = rr.center().y;
            let minus = Rect::from_center_size(pos2(rr.right() - 104.0, y), vec2(20.0, 22.0));
            let field = Rect::from_center_size(pos2(rr.right() - 73.0, y), vec2(38.0, 24.0));
            let plus = Rect::from_center_size(pos2(rr.right() - 42.0, y), vec2(20.0, 22.0));
            let mut q = p.qty;
            for (r, glyph, step) in [(minus, "−", -1i64), (plus, "+", 1i64)] {
                let resp = ui.interact(r, ui.id().with(("plasma_qty_step", i, step)), Sense::click());
                let enabled = !(step < 0 && q == 0);
                let fill = if enabled && resp.is_pointer_button_down_on() {
                    t.accent_soft
                } else if enabled && resp.hovered() {
                    t.hover
                } else {
                    t.control_fill
                };
                ui.painter().rect(r, t.control_radius, fill, Stroke::new(1.0, t.control_border), egui::StrokeKind::Inside);
                ui.painter().text(
                    r.center(),
                    Align2::CENTER_CENTER,
                    glyph,
                    crate::fonts::semibold(t.font_heading),
                    if enabled { t.text } else { t.text_dim },
                );
                crate::scenario::publish_target(&format!("plasma:qty:{i}:{}", if step < 0 { "minus" } else { "plus" }), r);
                if enabled && resp.clicked() {
                    q = (i64::from(q) + step).clamp(0, 9999) as u32;
                }
            }
            let mut qui = ui.new_child(
                egui::UiBuilder::new()
                    .max_rect(field)
                    .id_salt(("plasma_qty", i))
                    .layout(egui::Layout::centered_and_justified(egui::Direction::LeftToRight)),
            );
            qui.add(egui::DragValue::new(&mut q).range(0..=9999).speed(0.1));
            crate::scenario::publish_target(&format!("plasma:qty:{i}"), field);
            if q != p.qty {
                p.qty = q;
                changed = true;
            }
            let xr = Rect::from_center_size(pos2(rr.right() - 14.0, rr.center().y), vec2(20.0, 20.0));
            let xresp = ui.interact(xr, ui.id().with(("plasma_rm", i)), Sense::click());
            crate::icons::paint_ink(ui.painter(), xr.shrink(3.0), "close", if xresp.hovered() { t.error } else { t.text_dim });
            if xresp.clicked() {
                remove = Some(i);
            }
        }
    });
    if let Some(i) = remove {
        s.remove(i);
    }
    if changed {
        s.set_nest(None);
    }
    ui.add_space(t.space_8);
    ui.separator();
    ui.add_space(t.space_8);
    let tool = s.tool().map(|t| format!("{} · {}", t.metal.label(), t.name)).unwrap_or_default();
    heading(&mut ui, "Sheet", &tool);
    let (unit, k) = if s.metric { (" mm", MM_PER_IN) } else { (" in", 1.0) };
    let mut c = false;
    c |= scaled(&mut ui, "Sheet X", &mut s.sheet_w_in, k, 480.0, 0.05, unit);
    c |= scaled(&mut ui, "Sheet Y", &mut s.sheet_h_in, k, 480.0, 0.05, unit);
    c |= scaled(&mut ui, "Part spacing", &mut s.spacing_in, k, 4.0, 0.01, unit);
    c |= scaled(&mut ui, "Edge margin", &mut s.margin_in, k, 12.0, 0.01, unit);
    if c {
        s.set_nest(None);
    }
}

/// The tool library list, filtered; clicking a row picks the tool. Returns nothing; edits `s.tool`.
fn tool_list(ui: &mut egui::Ui, s: &mut State, height: f32, salt: &str, header: bool) {
    let t = Tokens::get();
    ui.horizontal_wrapped(|ui| {
        ui.spacing_mut().item_spacing = vec2(4.0, 4.0);
        for f in Filter::ALL {
            if ui.add(egui::Button::new(egui::RichText::new(crate::i18n::tr_label(f.label())).size(t.font_small)).selected(s.filter == f)).clicked() {
                s.filter = f;
                s.reveal_tool = true;
            }
        }
    });
    ui.add_space(t.space_4);
    if header {
        let (hr, _) = ui.allocate_exact_size(vec2(ui.available_width(), 16.0), Sense::hover());
        let font = FontId::proportional(t.font_small);
        ui.painter().text(pos2(hr.left() + 32.0, hr.center().y), Align2::LEFT_CENTER, crate::i18n::tr_label("Name"), font.clone(), t.text_dim);
        ui.painter().text(
            pos2(hr.right() - 56.0, hr.center().y),
            Align2::RIGHT_CENTER,
            crate::i18n::tr_label(if s.metric { "mm/min" } else { "IPM" }),
            font.clone(),
            t.text_dim,
        );
        ui.painter().text(pos2(hr.right() - 6.0, hr.center().y), Align2::RIGHT_CENTER, crate::i18n::tr_label("Kerf"), font, t.text_dim);
    }
    let reveal = std::mem::take(&mut s.reveal_tool);
    let filter = s.filter;
    let metric = s.metric;
    egui::ScrollArea::vertical().id_salt(salt).max_height(height).auto_shrink([false, false]).show(ui, |ui| {
        for (i, tool) in s.tools.iter().enumerate().filter(|(_, tl)| filter.keeps(tl)) {
            let (rr, resp) = ui.allocate_exact_size(vec2(ui.available_width(), 24.0), Sense::click());
            let sel = s.tool == i;
            if sel {
                ui.painter().rect_filled(rr, t.control_radius, t.accent_soft);
            } else if resp.hovered() {
                ui.painter().rect_filled(rr, t.control_radius, t.hover);
            }
            let ink = if sel { t.selected_ink } else { t.text };
            ui.painter().text(
                pos2(rr.left() + 6.0, rr.center().y),
                Align2::LEFT_CENTER,
                format!("{}", tool.number),
                FontId::proportional(t.font_small),
                t.text_dim,
            );
            let name = one_line(ui, tool.name.clone(), FontId::proportional(t.font_body), ink, (rr.width() - 120.0).max(20.0));
            ui.painter().galley(pos2(rr.left() + 32.0, rr.center().y - name.size().y / 2.0), name, ink);
            let feed = if metric { format!("{:.0}", tool.feed) } else { format!("{:.0}", tool.feed / MM_PER_IN) };
            ui.painter().text(pos2(rr.right() - 56.0, rr.center().y), Align2::RIGHT_CENTER, feed, FontId::proportional(t.font_body), ink);
            let kerf = if metric { format!("{:.2}", tool.kerf) } else { format!("{:.3}", tool.kerf / MM_PER_IN) };
            ui.painter().text(pos2(rr.right() - 6.0, rr.center().y), Align2::RIGHT_CENTER, kerf, FontId::proportional(t.font_body), t.text_dim);
            if sel && reveal {
                ui.scroll_to_rect(rr, Some(egui::Align::Center));
            }
            if resp.clicked() {
                s.tool = i;
            }
        }
    });
}

fn tools_panel(app: &mut SolveApp, ui: &mut egui::Ui, r: Rect) {
    let t = Tokens::get();
    let mut ui = child(ui, r, "plasma_tools");
    let s = &mut app.plasma_cam;
    heading(&mut ui, "Tool library", &format!("{} tools · Powermax 65", s.tools.len()));
    let list_h = (r.height() - 330.0).max(140.0);
    tool_list(&mut ui, s, list_h, "plasma_tool_rows", true);
    ui.add_space(t.space_8);
    ui.separator();
    ui.add_space(t.space_8);
    let metric = s.metric;
    let (unit, k) = if metric { (" mm", 1.0) } else { (" in", 1.0 / MM_PER_IN) };
    let idx = s.tool;
    let mut changed = false;
    if let Some(tool) = s.tools.get_mut(idx) {
        let thick = tool.thickness_in.map(|th| format!("{th:.4} in")).unwrap_or_else(|| "—".into());
        heading(&mut ui, &format!("Tool {}", tool.number), &format!("{} · {thick}", tool.metal.label()));
        changed |= scaled(&mut ui, "Feed rate", &mut tool.feed, k, 20_000.0, 1.0, if metric { " mm/min" } else { " ipm" });
        changed |= scaled(&mut ui, "Kerf width", &mut tool.kerf, k, 20.0, 0.01, unit);
        changed |= num(&mut ui, "Pierce delay", &mut tool.pierce_delay, 0.0..=120.0, 0.05, " s");
        changed |= scaled(&mut ui, "Pierce height", &mut tool.pierce_height, k, 50.0, 0.01, unit);
        changed |= scaled(&mut ui, "Cut height", &mut tool.cut_height, k, 50.0, 0.01, unit);
        changed |= num(&mut ui, "Cut volts", &mut tool.volts, 0.0..=300.0, 0.5, " V");
        changed |= num(&mut ui, "Wear per pierce", &mut tool.per_pierce, 0.0..=50.0, 0.01, " $");
    }
    if changed {
        s.posted = None;
    }
}

fn nest_panel(app: &mut SolveApp, ui: &mut egui::Ui, r: Rect) {
    let t = Tokens::get();
    let mut ui = child(ui, r, "plasma_nest");
    let s = &mut app.plasma_cam;
    heading(&mut ui, "Nest", "True shape");
    let mut pip = s.part_in_part;
    if crate::ui_kit::checkbox(&mut ui, "Small parts in holes", &mut pip).changed() {
        s.part_in_part = pip;
        s.set_nest(None);
    }
    ui.add_space(t.space_4);
    if s.busy() {
        ui.add_space(t.space_8);
        ui.horizontal(|ui| {
            ui.spinner();
            ui.label(egui::RichText::new(crate::i18n::tr_label("Nesting parts…")).size(t.font_body).color(t.text));
        });
        return;
    }
    let settings = s.settings();
    let Some(n) = s.nest.clone() else {
        note(&mut ui, "Press Nest Parts to lay the parts out on sheets by their real outlines.");
        return;
    };
    let total_area = n.sheets as f64 * settings.sheet_w * settings.sheet_h;
    let used: f64 = n.placements.iter().filter_map(|p| s.parts.get(p.part)).map(Part::area).sum();
    let tool = s.tool().cloned();
    stat(&mut ui, "Sheets", format!("{}", n.sheets));
    stat(&mut ui, "Parts placed", format!("{}", n.placements.len()));
    if !n.unplaced.is_empty() || n.truncated > 0 {
        stat(&mut ui, "Did not fit", format!("{}", n.unplaced.len() + n.truncated));
    }
    stat(&mut ui, "Overall utilisation", format!("{:.1} %", if total_area > 0.0 { used / total_area * 100.0 } else { 0.0 }));
    if let Some(tool) = &tool
        && let Some(th) = tool.thickness_in
    {
        stat(&mut ui, "Sheet steel", format!("{:.1} lb", total_area / (MM_PER_IN * MM_PER_IN) * th * tool.metal.density()));
    }
    stat(&mut ui, "Grid resolution", s.len(n.cell));
    ui.add_space(t.space_8);
    ui.separator();
    ui.add_space(t.space_8);
    heading(&mut ui, "Sheets", "");
    egui::ScrollArea::vertical().id_salt("plasma_sheet_rows").auto_shrink([false, false]).show(&mut ui, |ui| {
        for k in 0..n.sheets {
            let u = logic::utilisation(&s.parts, &n, k, &settings);
            let count = n.placements.iter().filter(|p| p.sheet == k).count();
            let length = logic::used_length(&s.parts, &n, k);
            let (rr, resp) = ui.allocate_exact_size(vec2(ui.available_width(), 40.0), Sense::click());
            if s.sheet == k {
                ui.painter().rect_filled(rr, t.control_radius, t.accent_soft);
            } else if resp.hovered() {
                ui.painter().rect_filled(rr, t.control_radius, t.hover);
            }
            ui.painter().text(
                pos2(rr.left() + 8.0, rr.top() + 12.0),
                Align2::LEFT_CENTER,
                format!("Sheet {}  ·  {count} parts  ·  uses {}", k + 1, s.len(length)),
                FontId::proportional(t.font_body),
                t.text,
            );
            let bar = Rect::from_min_size(pos2(rr.left() + 8.0, rr.top() + 26.0), vec2(rr.width() - 70.0, 5.0));
            ui.painter().rect_filled(bar, 2, t.panel_header);
            ui.painter().rect_filled(Rect::from_min_size(bar.min, vec2(bar.width() * u as f32, bar.height())), 2, t.accent);
            ui.painter().text(
                pos2(rr.right() - 8.0, rr.center().y),
                Align2::RIGHT_CENTER,
                format!("{:.0}%", u * 100.0),
                FontId::proportional(t.font_body),
                t.text_dim,
            );
            if resp.clicked() {
                s.sheet = k;
            }
        }
    });
}

/// A row of choice chips; returns true when the choice changed.
fn chips<T: PartialEq + Copy>(ui: &mut egui::Ui, label: &str, options: &[(T, &str)], value: &mut T) -> bool {
    let t = Tokens::get();
    let mut changed = false;
    crate::ui_kit::row(ui, label, |ui| {
        ui.horizontal(|ui| {
            ui.spacing_mut().item_spacing.x = 4.0;
            for (v, name) in options {
                if ui.add(egui::Button::new(egui::RichText::new(crate::i18n::tr_label(name)).size(t.font_small)).selected(*value == *v)).clicked()
                    && *value != *v
                {
                    *value = *v;
                    changed = true;
                }
            }
        });
    });
    changed
}

fn fmt_time(seconds: f64) -> String {
    let s = seconds.max(0.0).round() as u64;
    format!("{}:{:02}", s / 60, s % 60)
}

fn toolpath_panel(app: &mut SolveApp, ui: &mut egui::Ui, r: Rect) {
    let t = Tokens::get();
    let mut ui = child(ui, r, "plasma_toolpath");
    let s = &mut app.plasma_cam;
    let Some(tool) = s.tool().cloned() else { return };
    egui::ScrollArea::vertical().id_salt("plasma_toolpath_scroll").auto_shrink([false, false]).show(&mut ui, |ui| {
        ui.set_width(ui.available_width() - 6.0);
        heading(ui, "Toolpath", &format!("Tool {} · {}", tool.number, tool.name));
        // Cut side and direction, for the selected part or all parts.
        let targets = s.targets();
        let first = targets.first().and_then(|i| s.parts.get(*i)).map(|p| (p.side, p.reverse, p.flip_lead));
        if let Some((side0, rev0, flip0)) = first {
            heading(ui, "Cut side", &s.target_name());
            let mut side = side0;
            if chips(ui, "Offset", &[(Side::Outside, "Outside"), (Side::Inside, "Inside"), (Side::OnLine, "No offset")], &mut side) {
                for i in &targets {
                    if let Some(p) = s.parts.get_mut(*i) {
                        p.side = side;
                    }
                }
            }
            let mut rev = rev0;
            if crate::ui_kit::checkbox(ui, "Reverse cut direction", &mut rev).changed() {
                for i in &targets {
                    if let Some(p) = s.parts.get_mut(*i) {
                        p.reverse = rev;
                    }
                }
            }
            if side == Side::OnLine {
                let mut flip = flip0;
                if crate::ui_kit::checkbox(ui, "Reverse cut side (leads on the other side)", &mut flip).changed() {
                    for i in &targets {
                        if let Some(p) = s.parts.get_mut(*i) {
                            p.flip_lead = flip;
                        }
                    }
                }
            }
            note(
                ui,
                if rev {
                    "Reversed: outsides cut counter-clockwise, holes clockwise. With a clockwise-swirl torch the square edge is now on the scrap."
                } else {
                    "Outsides cut clockwise, holes counter-clockwise: with a clockwise-swirl torch (Hypertherm) the square edge lands on the part."
                },
            );
            note(ui, "Click a part in the list to set it alone; with none selected these apply to every part.");
            ui.add_space(t.space_6);
        }
        let (unit, k) = if s.metric { (" mm", 1.0) } else { (" in", 1.0 / MM_PER_IN) };
        let mut p = s.post;
        heading(ui, "Leads", "");
        let leads: Vec<(Lead, &str)> = Lead::ALL.iter().map(|l| (*l, l.label())).collect();
        chips(ui, "Lead-in", &leads, &mut p.lead_in_kind);
        if p.lead_in_kind != Lead::None {
            scaled(ui, if p.lead_in_kind == Lead::Arc { "Lead-in radius" } else { "Lead-in length" }, &mut p.lead_in, k, 50.0, 0.1, unit);
        }
        chips(ui, "Lead-out", &leads, &mut p.lead_out_kind);
        if p.lead_out_kind != Lead::None {
            scaled(ui, if p.lead_out_kind == Lead::Arc { "Lead-out radius" } else { "Lead-out length" }, &mut p.lead_out, k, 50.0, 0.1, unit);
        }
        scaled(ui, "Overcut", &mut p.overcut, k, 50.0, 0.1, unit);
        ui.add_space(t.space_6);
        heading(ui, "Holes and output", "");
        scaled(ui, "Slow holes under", &mut p.small_hole, k, 500.0, 0.5, unit);
        num(ui, "Hole speed", &mut p.hole_speed_pct, 10.0..=100.0, 1.0, " %");
        scaled(ui, "Rapid speed", &mut p.rapid, k, 50_000.0, 10.0, if s.metric { " mm/min" } else { " ipm" });
        crate::ui_kit::checkbox(ui, "Inch program (G20) - match the QtPlasmaC config", &mut p.inch);
        crate::ui_kit::checkbox(ui, "Select the QtPlasmaC material (M190)", &mut p.use_material);
        crate::ui_kit::checkbox(ui, "Kerf compensation in QtPlasmaC (G41.1 / G42.1)", &mut p.kerf_comp);
        if !p.kerf_comp {
            let half = s.tool().map_or(0.0, |t| t.kerf) / 2.0;
            let off = if s.metric { format!("{half:.2} mm") } else { format!("{:.4} in", half / MM_PER_IN) };
            note(ui, &format!("Kerf offset in the toolpath, as SheetCAM does: the torch runs {off} off the line."));
        }
        s.post = p;
        if s.busy() {
            ui.horizontal(|ui| {
                ui.spinner();
                ui.label(egui::RichText::new(crate::i18n::tr_label("Nesting parts…")).size(t.font_body).color(t.text));
            });
            return;
        }
        if s.nest.is_none() {
            note(ui, "Nest the parts (Nest tab) and the toolpath is built from the nest.");
            return;
        }
        let metric = s.metric;
        let len_fmt = |mm: f64| if metric { format!("{mm:.0} mm") } else { format!("{:.1} in", mm / MM_PER_IN) };
        let Some(posted) = s.posted() else { return };
        let cut: f64 = posted.cuts.iter().map(Cut::length).sum();
        let pierces = posted.cuts.len();
        let total = posted.total;
        let arcs = posted.code.lines().map(str::trim_start).filter(|l| l.starts_with("G2 ") || l.starts_with("G3 ")).count();
        let lines = posted.code.lines().count();
        let preview: String = posted.code.lines().take(80).collect::<Vec<_>>().join("\n");
        ui.add_space(t.space_6);
        ui.separator();
        ui.add_space(t.space_6);
        stat(ui, "Cut length", len_fmt(cut));
        stat(ui, "Pierces", format!("{pierces}"));
        stat(ui, "Run time", fmt_time(total));
        stat(ui, "Arc moves (G2/G3)", format!("{arcs}"));
        ui.add_space(t.space_6);
        heading(ui, "G-code preview", &format!("{lines} lines"));
        egui::ScrollArea::vertical().id_salt("plasma_gcode").max_height(260.0).auto_shrink([false, true]).show(ui, |ui| {
            ui.label(egui::RichText::new(preview).monospace().size(t.font_small).color(t.text));
        });
    });
}

/// A one-line text field in a form row.
fn text_row(ui: &mut egui::Ui, label: &str, value: &mut String, hint: &str) -> bool {
    crate::ui_kit::row(ui, label, |ui| {
        let resp = ui.add(egui::TextEdit::singleline(value).hint_text(crate::i18n::tr_label(hint)).desired_width(ui.available_width()));
        Tokens::get().input_bevel(ui, &resp);
        resp.changed()
    })
}

/// A labelled multi-line text box.
fn text_box(ui: &mut egui::Ui, label: &str, value: &mut String, rows: usize, hint: &str) -> bool {
    let t = Tokens::get();
    ui.label(egui::RichText::new(crate::i18n::tr_label(label)).size(t.font_label).color(t.dialog_label));
    let resp = ui.add(egui::TextEdit::multiline(value).desired_rows(rows).hint_text(crate::i18n::tr_label(hint)).desired_width(ui.available_width()));
    t.input_bevel(ui, &resp);
    ui.add_space(t.space_4);
    resp.changed()
}

fn quote_panel(app: &mut SolveApp, ui: &mut egui::Ui, r: Rect) {
    let t = Tokens::get();
    let mut ui = child(ui, r, "plasma_quote");
    let mut pick_logo = false;
    let s = &mut app.plasma_cam;
    egui::ScrollArea::vertical().id_salt("plasma_quote_scroll").auto_shrink([false, false]).show(&mut ui, |ui| {
        ui.set_width(ui.available_width() - 6.0);
        let tool_name = s.tool().map(|t| t.name.clone()).unwrap_or_default();
        heading(ui, "Quote", &tool_name);
        tool_list(ui, s, 120.0, "plasma_quote_tools", false);
        ui.add_space(t.space_6);
        let Some(tool) = s.tool().cloned() else { return };
        let p = &mut s.pricing;
        heading(ui, "Prices", "");
        match tool.metal {
            Metal::Stainless => num(ui, "Stainless price", &mut p.stainless_per_lb, 0.0..=50.0, 0.01, " $/lb"),
            Metal::Aluminum => num(ui, "Aluminum price", &mut p.aluminum_per_lb, 0.0..=50.0, 0.01, " $/lb"),
            Metal::Steel | Metal::Other => num(ui, "Steel price", &mut p.steel_per_lb, 0.0..=50.0, 0.01, " $/lb"),
        };
        num(ui, "Price per pierce", &mut p.per_pierce, 0.0..=100.0, 0.01, " $");
        num(ui, "Price per cut inch", &mut p.per_inch, 0.0..=100.0, 0.005, " $/in");
        num(ui, "Machine rate", &mut p.machine_per_hour, 0.0..=1000.0, 1.0, " $/hr");
        num(ui, "Setup fee", &mut p.setup, 0.0..=10000.0, 1.0, " $");
        num(ui, "Scrap allowance", &mut p.scrap_allowance_pct, 0.0..=200.0, 1.0, " %");
        num(ui, "Markup", &mut p.markup_pct, 0.0..=500.0, 1.0, " %");
        num(ui, "Sales tax", &mut p.tax_pct, 0.0..=50.0, 0.05, " %");
        let mut full = p.charge_full_sheets;
        if crate::ui_kit::checkbox(ui, "Charge full sheets (needs a nest)", &mut full).changed() {
            p.charge_full_sheets = full;
        }
        note(ui, "Set the machine rate to 0 to price only by pierces and cut inches.");
        if tool.thickness_in.is_none() {
            note(ui, "This tool's name has no thickness, so material is not priced.");
        }
        let q = logic::quote(&s.parts, &tool, &s.pricing, sheets_area(s));
        ui.add_space(t.space_4);
        ui.separator();
        ui.add_space(t.space_4);
        stat(ui, "Cut length · pierces", format!("{:.1} in · {}", q.cut_in, q.pierces));
        stat(ui, "Machine time", format!("{:.1} min", q.minutes));
        stat(ui, "Part weight", format!("{:.1} lb", q.weight_lb));
        stat(ui, "Material", format!("$ {:.2}", q.material));
        if q.pierce_charge > 0.0 {
            stat(ui, "Pierces", format!("$ {:.2}", q.pierce_charge));
        }
        if q.inch_charge > 0.0 {
            stat(ui, "Cut inches", format!("$ {:.2}", q.inch_charge));
        }
        stat(ui, "Machine + consumables", format!("$ {:.2}", q.machine + q.consumables));
        stat(ui, "Setup + markup", format!("$ {:.2}", q.setup + q.markup));
        let tax = q.total * s.pricing.tax_pct.max(0.0) / 100.0;
        if tax > 0.0 {
            stat(ui, "Sales tax", format!("$ {tax:.2}"));
        }
        ui.add_space(t.space_4);
        let (tr, _) = ui.allocate_exact_size(vec2(ui.available_width(), 38.0), Sense::hover());
        ui.painter().rect_filled(tr, t.control_radius, t.accent_soft);
        ui.painter().text(
            pos2(tr.left() + 10.0, tr.center().y),
            Align2::LEFT_CENTER,
            crate::i18n::tr_label("Total"),
            crate::fonts::semibold(t.font_heading),
            t.selected_ink,
        );
        ui.painter().text(
            pos2(tr.right() - 10.0, tr.center().y),
            Align2::RIGHT_CENTER,
            format!("$ {:.2}", q.total + tax),
            crate::fonts::semibold(t.font_heading + 2.0),
            t.selected_ink,
        );
        ui.add_space(t.space_6);
        for (i, l) in q.lines.iter().enumerate() {
            let colour = s.colour(i);
            ui.horizontal(|ui| {
                let (dot, _) = ui.allocate_exact_size(vec2(10.0, 10.0), Sense::hover());
                ui.painter().circle_filled(dot.center(), 4.0, colour);
                ui.label(egui::RichText::new(format!("{} × {}", l.qty, l.name)).size(t.font_small).color(t.text));
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    ui.label(egui::RichText::new(format!("$ {:.2}  ($ {:.2} ea)", l.total, l.each)).size(t.font_small).color(t.text_dim));
                });
            });
        }
        ui.add_space(t.space_8);
        ui.separator();
        // The customer's quote page.
        egui::CollapsingHeader::new(egui::RichText::new(crate::i18n::tr_label("Customer and job")).semibold())
            .id_salt("plasma_quote_customer")
            .default_open(true)
            .show(ui, |ui| {
                let c = &mut s.customer;
                text_row(ui, "Customer", &mut c.name, "Company or name");
                text_row(ui, "Contact", &mut c.contact, "Attention");
                text_box(ui, "Customer address", &mut c.address, 2, "Street, city, state, ZIP");
                text_row(ui, "Phone", &mut c.phone, "");
                text_row(ui, "Email", &mut c.email, "");
                text_row(ui, "Job name", &mut c.job_name, "");
                text_row(ui, "Job / quote #", &mut c.job_number, "");
                text_row(ui, "Customer PO", &mut c.po, "");
                let mut days = c.valid_days;
                crate::ui_kit::row(ui, "Valid for", |ui| ui.add(egui::DragValue::new(&mut days).range(1..=365).suffix(" days")));
                c.valid_days = days;
                text_box(ui, "Notes on this quote", &mut c.notes, 2, "Shown on the printed quote");
            });
        egui::CollapsingHeader::new(egui::RichText::new(crate::i18n::tr_label("Your business")).semibold())
            .id_salt("plasma_quote_business")
            .default_open(s.business.company.is_empty())
            .show(ui, |ui| {
                let b = &mut s.business;
                text_row(ui, "Company", &mut b.company, "Your company name");
                text_box(ui, "Address", &mut b.address, 2, "Street, city, state, ZIP");
                text_row(ui, "Phone", &mut b.phone, "");
                text_row(ui, "Email", &mut b.email, "");
                text_row(ui, "Website", &mut b.website, "");
                crate::ui_kit::row(ui, "Logo", |ui| {
                    if ui.button(crate::i18n::tr_label(if b.logo.is_empty() { "Choose image…" } else { "Change…" })).clicked() {
                        pick_logo = true;
                    }
                    if !b.logo.is_empty() {
                        let name = b.logo.rsplit(['/', '\\']).next().unwrap_or(&b.logo).to_string();
                        ui.label(egui::RichText::new(name).size(t.font_small).color(t.text_dim));
                        if ui.small_button("×").on_hover_text(crate::i18n::tr_label("Remove the logo")).clicked() {
                            b.logo.clear();
                        }
                    }
                });
                text_box(ui, "Terms", &mut b.terms, 3, "Payment terms, lead time, pickup or delivery…");
                note(ui, "Your business details and prices are saved for next time.");
            });
    });
    s.autosave();
    if pick_logo {
        app.pick_open_for(&["png", "jpg", "jpeg", "gif", "svg", "webp"], |app, path| {
            if let Some(path) = path {
                app.plasma_cam.business.logo = path;
                app.plasma_cam.autosave();
            }
        });
    }
}

/// The customer quote page for the current job.
fn quote_page(s: &State) -> Option<(String, String)> {
    let tool = s.tool()?;
    let q = logic::quote(&s.parts, tool, &s.pricing, sheets_area(s));
    let logo = if s.business.logo.is_empty() {
        None
    } else {
        solidcraft_engine::io::vfs::read(&s.business.logo, 8 << 20).ok().and_then(|bytes| quote::logo_uri(&s.business.logo, &bytes))
    };
    let date = quote::today();
    let page = quote::html(&s.business, &s.customer, tool, &q, s.pricing.tax_pct, logo.as_deref(), date);
    let tag: String = if s.customer.job_number.trim().is_empty() {
        format!("{}-{:02}-{:02}", date.0, date.1, date.2)
    } else {
        s.customer.job_number.trim().to_string()
    };
    let tag: String = tag.chars().map(|c| if c.is_ascii_alphanumeric() || c == '-' || c == '_' { c } else { '-' }).collect();
    Some((format!("Quote-{tag}.html"), page))
}

/// Write the quote page and open it in the browser to print or save as PDF.
fn print_quote(app: &mut SolveApp, ctx: &egui::Context) {
    let s = &mut app.plasma_cam;
    let Some((name, page)) = quote_page(s) else { return };
    #[cfg(not(target_arch = "wasm32"))]
    {
        let Some(dir) = quote::quotes_dir() else {
            s.message = Some(("No folder to write the quote to.".into(), true));
            return;
        };
        let path = dir.join(&name);
        let written = std::fs::create_dir_all(&dir).and_then(|_| std::fs::write(&path, page.as_bytes()));
        match written {
            Ok(()) => {
                let url = format!("file:///{}", path.to_string_lossy().replace('\\', "/").trim_start_matches('/'));
                ctx.open_url(egui::OpenUrl::new_tab(url));
                s.message = Some((format!("Opened {name} in your browser: print it or save it as PDF."), false));
            }
            Err(e) => s.message = Some((format!("Could not write the quote: {e}"), true)),
        }
    }
    #[cfg(target_arch = "wasm32")]
    {
        let _ = (ctx, name, page);
        s.message = Some(("Use Save Quote in the browser version.".into(), true));
    }
}

fn save_quote(app: &mut SolveApp) {
    let Some((name, page)) = quote_page(&app.plasma_cam) else { return };
    app.pick_save(&name, &["html"], move |app, path| {
        let Some(path) = path else { return };
        app.plasma_cam.message = Some(match solidcraft_engine::io::vfs::write(&path, page.as_bytes()) {
            Ok(()) => (format!("Saved {path}"), false),
            Err(e) => (format!("Could not save the quote: {e}"), true),
        });
    });
}

/// Contours shaded with their holes cut out, through `map` (mm → screen).
fn paint_part(
    painter: &egui::Painter,
    contours: &[Contour],
    tris: &[Vec<[usize; 3]>],
    map: &dyn Fn([f64; 2]) -> egui::Pos2,
    ink: Color32,
    hole_fill: Color32,
    width: f32,
) {
    let fill = ink.gamma_multiply(0.28);
    for (ci, c) in contours.iter().enumerate() {
        let mut mesh = egui::Mesh::default();
        let colour = if ci == 0 { fill } else { hole_fill };
        for q in &c.pts {
            mesh.colored_vertex(map(*q), colour);
        }
        for tri in tris.get(ci).map(Vec::as_slice).unwrap_or(&[]) {
            let [a, b, c] = *tri;
            mesh.add_triangle(a as u32, b as u32, c as u32);
        }
        painter.add(Shape::mesh(mesh));
    }
    for c in contours {
        painter.add(Shape::closed_line(c.pts.iter().map(|q| map(*q)).collect(), Stroke::new(width, ink)));
    }
}

fn sheet_canvas(app: &mut SolveApp, ui: &mut egui::Ui, r: Rect) {
    let t = Tokens::get();
    let head = Rect::from_min_size(r.min, vec2(r.width(), 34.0));
    let foot = Rect::from_min_max(pos2(r.left(), r.bottom() - 28.0), r.max);
    let canvas = Rect::from_min_max(pos2(r.left(), head.bottom()), pos2(r.right(), foot.top() - 4.0));
    // Frame the sheet when it changes size, or when asked.
    {
        let s = &mut app.plasma_cam;
        let key = (s.sheet_w_in.to_bits(), s.sheet_h_in.to_bits());
        if s.view_key != Some(key) {
            s.view_key = Some(key);
            s.fitted = false;
        }
        if !s.fitted {
            s.fit(canvas);
        }
    }
    // Picking and moving parts by hand comes before navigation, so a press on a part moves the part.
    let resp = ui.interact(canvas, ui.id().with("plasma_sheet_canvas"), Sense::click_and_drag());
    let (pointer, pressed, down, escape) =
        ui.input(|i| (i.pointer.hover_pos(), i.pointer.primary_pressed(), i.pointer.primary_down(), i.key_pressed(egui::Key::Escape)));
    let sim_bar = Rect::from_min_size(pos2(canvas.left(), canvas.bottom() - 52.0), vec2(canvas.width(), 52.0));
    let over = pointer.filter(|p| canvas.contains(*p) && !crate::shell::covered(app, *p) && !(app.plasma_cam.sim.is_some() && sim_bar.contains(*p)));
    let view = SheetView::of(&app.plasma_cam.cam, canvas);
    let settings = app.plasma_cam.settings();
    let under = {
        let s = &app.plasma_cam;
        let can_move = s.sim.is_none() && !s.busy();
        match (&s.nest, over) {
            (Some(n), Some(p)) if can_move => logic::placement_at(&s.parts, n, s.sheet, view.sheet_at(p)),
            _ => None,
        }
    };
    {
        let s = &mut app.plasma_cam;
        if pressed
            && s.drag.is_none()
            && let (Some(i), Some(p)) = (under, over)
            && let Some(pl) = s.nest.as_ref().and_then(|n| n.placements.get(i))
        {
            s.drag = Some(Drag { placement: i, grab: view.sheet_at(p), from: (pl.tx, pl.ty) });
        }
        if let Some(d) = s.drag {
            let restore = |s: &mut State, at: (f64, f64)| {
                if let Some(pl) = s.nest.as_mut().and_then(|n| n.placements.get_mut(d.placement)) {
                    pl.tx = at.0;
                    pl.ty = at.1;
                }
            };
            if escape {
                restore(s, d.from);
                s.drag = None;
                s.clash = None;
            } else {
                if let Some(p) = pointer {
                    let q = view.sheet_at(p);
                    restore(s, (d.from.0 + q[0] - d.grab[0], d.from.1 + q[1] - d.grab[1]));
                }
                let (hits, off) = s.nest.as_ref().map(|n| logic::contacts(&s.parts, n, d.placement, &settings)).unwrap_or_default();
                s.clash = (!hits.is_empty() || off).then_some((d.placement, hits, off));
                if !down {
                    // Dropped: the toolpath follows the new layout.
                    s.drag = None;
                    s.nest_rev += 1;
                    s.posted = None;
                    let name = s
                        .nest
                        .as_ref()
                        .and_then(|n| n.placements.get(d.placement))
                        .and_then(|pl| s.parts.get(pl.part))
                        .map(|p| p.name.clone())
                        .unwrap_or_default();
                    s.message = Some(match &s.clash {
                        Some((_, _, off)) if *off => (format!("{name} is off the usable sheet: move it back inside the margin."), true),
                        Some((_, hits, _)) => (format!("{name} touches {} other part(s): move it further away.", hits.len()), true),
                        None => (format!("Moved {name}."), false),
                    });
                }
            }
        }
    }
    // A part left touching after a drop stays red until it, or what it touched, is moved clear.
    {
        let s = &mut app.plasma_cam;
        if s.drag.is_none()
            && let Some((i, _, _)) = s.clash
            && let Some(n) = &s.nest
        {
            let (hits, off) = logic::contacts(&s.parts, n, i, &settings);
            s.clash = (!hits.is_empty() || off).then_some((i, hits, off));
        }
    }
    let holding = app.plasma_cam.drag.is_some();
    // Zoom with the wheel and pan through SolidCraft's shared navigation (no orbit on a flat page).
    if !holding && !(pressed && under.is_some()) {
        let design = std::mem::replace(&mut app.cam, app.plasma_cam.cam);
        let _ = crate::navigation::handle(app, ui, canvas, &resp, crate::navigation::NavCaps { orbit: false, pick: false });
        app.plasma_cam.cam = std::mem::replace(&mut app.cam, design);
        app.viewport.mouse = None;
    }
    if holding {
        ui.ctx().set_cursor_icon(egui::CursorIcon::Grabbing);
    } else if under.is_some() {
        ui.ctx().set_cursor_icon(egui::CursorIcon::Grab);
    }
    let view = SheetView::of(&app.plasma_cam.cam, canvas);
    let map = move |q: [f64; 2]| view.map(q);
    let s = &mut app.plasma_cam;
    let sheets = s.nest.as_ref().map_or(1, |n| n.sheets.max(1));
    s.sheet = s.sheet.min(sheets - 1);
    // Header.
    let title = if s.busy() {
        "Nesting…".to_string()
    } else if s.nest.is_some() {
        format!("Sheet {} of {sheets}", s.sheet + 1)
    } else {
        "Sheet (not nested)".to_string()
    };
    let title_g = ui.painter().layout_no_wrap(title, crate::fonts::semibold(t.font_heading), t.text);
    let title_w = title_g.size().x;
    ui.painter().galley(pos2(head.left() + 4.0, head.center().y - title_g.size().y / 2.0), title_g, t.text);
    let fit = ui.put(
        Rect::from_min_size(pos2(head.left() + title_w + 18.0, head.center().y - 12.0), vec2(84.0, 24.0)),
        egui::Button::new(crate::i18n::tr_label("Fit sheet")),
    );
    crate::scenario::publish_target("plasma:fit", fit.rect);
    if fit.clicked() {
        s.fitted = false;
    }
    let tool = s.tool().map(|tl| tl.name.clone()).unwrap_or_default();
    let dims = format!("{} × {}  ·  {tool}", s.len(settings.sheet_w), s.len(settings.sheet_h));
    ui.painter().text(pos2(head.right() - 4.0, head.center().y), Align2::RIGHT_CENTER, dims, FontId::proportional(t.font_body), t.text_dim);
    // Everything on the sheet stays inside the canvas when zoomed in.
    let outer_clip = ui.clip_rect();
    ui.set_clip_rect(canvas.intersect(outer_clip));
    let (sw, sh) = (settings.sheet_w.max(1.0), settings.sheet_h.max(1.0));
    let sheet_rect = Rect::from_two_pos(map([0.0, 0.0]), map([sw, sh]));
    ui.painter().rect_filled(sheet_rect.translate(vec2(3.0, 4.0)), 2, t.overlay.gamma_multiply(0.25));
    ui.painter().rect(sheet_rect, 2, t.panel, Stroke::new(1.0, t.border), egui::StrokeKind::Outside);
    let m = settings.margin;
    if m > 0.0 && 2.0 * m < sw.min(sh) {
        let usable = Rect::from_two_pos(map([m, m]), map([sw - m, sh - m]));
        for shape in Shape::dashed_line(
            &[usable.left_top(), usable.right_top(), usable.right_bottom(), usable.left_bottom(), usable.left_top()],
            Stroke::new(1.0, t.grid_major),
            6.0,
            4.0,
        ) {
            ui.painter().add(shape);
        }
    }
    ui.painter().circle_filled(map([0.0, 0.0]), 3.0, t.axis_x);
    let mut hovered: Option<String> = None;
    let toolpath = s.page == Page::Toolpath;
    let grey = t.text_dim;
    let dragging = s.drag.map(|d| d.placement);
    let red: Vec<usize> = match &s.clash {
        Some((me, hits, _)) => std::iter::once(*me).chain(hits.iter().copied()).collect(),
        None => Vec::new(),
    };
    if let Some(n) = s.nest.clone() {
        for (i, pl) in n.placements.iter().enumerate().filter(|(_, p)| p.sheet == s.sheet) {
            let (Some(part), Some(tris)) = (s.parts.get(pl.part), s.tris.get(pl.part)) else { continue };
            let placed: Vec<Contour> = part.contours.iter().map(|c| pl.contour(c)).collect();
            let lit = dragging == Some(i) || (dragging.is_none() && under == Some(i));
            // Each drawing keeps its own colour while nesting; once the toolpath is up they turn grey. A part
            // touching another after a move turns red.
            let colour = if red.contains(&i) {
                t.error
            } else if lit || (!toolpath && s.selected == Some(pl.part)) {
                // On the Toolpath tab a selected part stays grey so its cut path reads; the list shows the selection.
                t.sel_edge
            } else if toolpath {
                grey
            } else {
                s.colour(pl.part)
            };
            if lit {
                hovered = Some(format!(
                    "{}  ·  {} × {}  ·  {:.0}°  ·  drag to move, Esc to cancel",
                    part.name,
                    s.len(part.width),
                    s.len(part.height),
                    pl.angle_deg
                ));
            }
            paint_part(ui.painter(), &placed, tris, &map, colour, t.panel, if lit || red.contains(&i) { 2.2 } else { 1.2 });
        }
        if toolpath && s.drag.is_none() && s.posted().is_some() {
            // Borrow the cached toolpath out of the state while drawing, then put it back.
            let posted = s.posted.take();
            if let Some(posted) = &posted {
                let mut sim = s.sim;
                draw_toolpath(ui, posted, &map, &mut sim, canvas);
                s.sim = sim;
            }
            s.posted = posted;
        }
    } else {
        let text = if s.busy() {
            "Nesting parts…"
        } else if !s.parts.is_empty() {
            "Press Nest Parts to lay out the parts"
        } else {
            "Import DXF parts to begin"
        };
        ui.painter().text(sheet_rect.center(), Align2::CENTER_CENTER, crate::i18n::tr_label(text), FontId::proportional(t.font_heading), t.text_dim);
    }
    ui.set_clip_rect(outer_clip);
    crate::scenario::publish_rect("plasma:sheet", sheet_rect);
    let (text, err) = if let Some((_, hits, off)) = &s.clash
        && s.drag.is_some()
    {
        if *off {
            ("Off the usable sheet".to_string(), true)
        } else {
            (format!("Too close: touching {} part(s) - keep {} apart", hits.len(), s.len(settings.spacing)), true)
        }
    } else if let Some(h) = hovered {
        (h, false)
    } else if let Some((m, e)) = &s.message {
        (m.clone(), *e)
    } else {
        ("Scroll to zoom · middle-drag to pan · drag a part to move it".to_string(), false)
    };
    let mut right = String::new();
    if let Some(n) = &s.nest {
        let u = logic::utilisation(&s.parts, n, s.sheet, &settings);
        let count = n.placements.iter().filter(|p| p.sheet == s.sheet).count();
        right = format!("{count} parts  ·  utilisation {:.1}%", u * 100.0);
    }
    ui.painter().text(
        pos2(foot.left() + 4.0, foot.center().y),
        Align2::LEFT_CENTER,
        text,
        FontId::proportional(t.font_body),
        if err { t.error } else { t.text_dim },
    );
    ui.painter().text(pos2(foot.right() - 4.0, foot.center().y), Align2::RIGHT_CENTER, right, FontId::proportional(t.font_body), t.text_dim);
}

/// A small arrow at `p` pointing along `dir` (screen space).
fn arrow(painter: &egui::Painter, p: egui::Pos2, dir: egui::Vec2, colour: Color32) {
    let d = dir.normalized();
    if !d.x.is_finite() {
        return;
    }
    let n = egui::vec2(-d.y, d.x);
    let tip = p + d * 5.0;
    painter.add(Shape::convex_polygon(vec![tip, p - d * 4.0 + n * 4.0, p - d * 4.0 - n * 4.0], colour, Stroke::NONE));
}

/// The toolpath on the sheet: rapids dashed, cuts in the accent, leads in amber with the pierce marked, an arrow
/// for each cut's direction. With the simulation on, the torch runs along it and what it has cut turns red.
fn draw_toolpath(ui: &mut egui::Ui, posted: &Posted, map: &dyn Fn([f64; 2]) -> egui::Pos2, sim: &mut Option<Sim>, canvas: Rect) {
    let t = Tokens::get();
    let painter = ui.painter().clone();
    let faint = sim.is_some();
    let rapid_ink = t.text_dim.gamma_multiply(if faint { 0.35 } else { 0.7 });
    let cut_ink = if faint { t.accent.gamma_multiply(0.35) } else { t.accent };
    let lead_ink = if faint { t.warning.gamma_multiply(0.4) } else { t.warning };
    let screen = |pts: &[[f64; 2]]| pts.iter().map(|q| map(*q)).collect::<Vec<_>>();
    let mut last = map([0.0, 0.0]);
    for (i, cut) in posted.cuts.iter().enumerate() {
        let (pts, a, b) = cut.polyline();
        let pierce = map(cut.pierce);
        for shape in Shape::dashed_line(&[last, pierce], Stroke::new(1.0, rapid_ink), 4.0, 3.0) {
            painter.add(shape);
        }
        if let (Some(lead), Some(body), Some(tail)) = (pts.get(..=a), pts.get(a..=b), pts.get(b..)) {
            painter.add(Shape::line(screen(lead), Stroke::new(2.0, lead_ink)));
            painter.add(Shape::line(screen(body), Stroke::new(1.6, cut_ink)));
            painter.add(Shape::line(screen(tail), Stroke::new(2.0, lead_ink)));
            // Direction: an arrow a fifth of the way round.
            let (q, _) = logic::along(body, 0.2);
            let (q2, _) = logic::along(body, 0.21);
            arrow(&painter, map(q), map(q2) - map(q), cut_ink);
        }
        painter.circle_filled(pierce, 2.5, lead_ink);
        if !cut.hole && !faint {
            let (lo, hi) = cut.contour.bounds();
            painter.text(
                map([(lo[0] + hi[0]) / 2.0, (lo[1] + hi[1]) / 2.0]),
                Align2::CENTER_CENTER,
                format!("{}", i + 1),
                FontId::proportional(t.font_small),
                t.text_dim,
            );
        }
        last = map(cut.end());
    }
    let Some(state) = sim.as_mut() else { return };
    // Advance the clock.
    if state.playing {
        let dt = f64::from(ui.input(|i| i.stable_dt).min(0.1));
        state.t += dt * state.speed;
        if state.t >= posted.total {
            state.t = posted.total;
            state.playing = false;
        }
        ui.ctx().request_repaint();
    }
    // What the torch has done by `t`, and where it is.
    let burnt = t.error;
    let mut clock = 0.0;
    let mut torch = map([0.0, 0.0]);
    let mut firing = false;
    for m in &posted.motions {
        let end = clock + m.seconds;
        let frac = if state.t >= end {
            1.0
        } else if state.t <= clock {
            0.0
        } else {
            (state.t - clock) / m.seconds.max(1e-9)
        };
        if frac > 0.0 || (state.t >= clock && m.seconds <= 0.0) {
            let (p, done) = logic::along(&m.pts, frac);
            match m.kind {
                MoveKind::Cut => {
                    painter.add(Shape::line(screen(&done), Stroke::new(2.4, burnt)));
                    firing = frac < 1.0;
                }
                MoveKind::Rapid => {
                    for shape in Shape::dashed_line(&screen(&done), Stroke::new(1.0, t.text_dim), 4.0, 3.0) {
                        painter.add(shape);
                    }
                    firing = false;
                }
                MoveKind::Pierce => {
                    painter.circle_stroke(map(p), 3.5, Stroke::new(1.5, burnt));
                    firing = frac < 1.0;
                }
            }
            torch = map(p);
        }
        if state.t < end {
            break;
        }
        clock = end;
    }
    // The torch.
    if firing {
        painter.circle_filled(torch, 9.0, t.warning.gamma_multiply(0.35));
    }
    painter.circle_stroke(torch, 6.0, Stroke::new(2.0, t.text));
    painter.circle_filled(torch, 2.5, if firing { t.warning } else { t.text });
    // Player: play/pause, restart, speed, a scrubbable progress bar and the clock.
    let bar = Rect::from_min_size(pos2(canvas.left() + 16.0, canvas.bottom() - 48.0), vec2(canvas.width() - 32.0, 38.0));
    painter.rect(bar, t.radius_lg, t.panel, Stroke::new(1.0, t.border), egui::StrokeKind::Inside);
    let mut row = ui.new_child(
        egui::UiBuilder::new()
            .max_rect(bar.shrink2(vec2(10.0, 6.0)))
            .id_salt("plasma_sim_bar")
            .layout(egui::Layout::left_to_right(egui::Align::Center)),
    );
    row.spacing_mut().item_spacing.x = 6.0;
    let play_label = if state.playing {
        "Pause"
    } else if state.t >= posted.total {
        "Replay"
    } else {
        "Play"
    };
    let play = row.add(egui::Button::new(crate::i18n::tr_label(play_label)).min_size(vec2(64.0, 24.0)));
    crate::scenario::publish_target("plasma:sim:play", play.rect);
    if play.clicked() {
        if state.t >= posted.total {
            state.t = 0.0;
        }
        state.playing = !state.playing;
    }
    if row.add(egui::Button::new(crate::i18n::tr_label("Restart")).min_size(vec2(64.0, 24.0))).clicked() {
        state.t = 0.0;
        state.playing = true;
    }
    for sp in SIM_SPEEDS {
        let b = row.add(egui::Button::new(egui::RichText::new(format!("{sp}×")).size(t.font_small)).selected((state.speed - sp).abs() < 1e-9));
        crate::scenario::publish_target(&format!("plasma:sim:speed:{sp}"), b.rect);
        if b.clicked() {
            state.speed = sp;
        }
    }
    let clock_text = format!("{} / {}", fmt_time(state.t), fmt_time(posted.total));
    let text_w = 90.0;
    let track_w = (row.available_width() - text_w - 8.0).max(40.0);
    let (track, resp) = row.allocate_exact_size(vec2(track_w, 24.0), Sense::click_and_drag());
    let line = Rect::from_center_size(track.center(), vec2(track.width(), 6.0));
    painter.rect_filled(line, 3, t.panel_header);
    let frac = if posted.total > 0.0 { (state.t / posted.total).clamp(0.0, 1.0) as f32 } else { 0.0 };
    painter.rect_filled(Rect::from_min_size(line.min, vec2(line.width() * frac, line.height())), 3, burnt);
    painter.circle_filled(pos2(line.left() + line.width() * frac, line.center().y), 6.0, burnt);
    if let Some(pos) = resp.interact_pointer_pos()
        && (resp.dragged() || resp.clicked())
    {
        state.t = f64::from(((pos.x - line.left()) / line.width().max(1.0)).clamp(0.0, 1.0)) * posted.total;
    }
    row.label(egui::RichText::new(clock_text).size(t.font_body).color(t.text));
}
