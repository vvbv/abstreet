use abstutil::Timer;
use ezgui::{
    hotkey, Color, EventCtx, EventLoopMode, GfxCtx, Key, ModalMenu, Text, WarpingItemSlider, GUI,
};
use geom::{Circle, Distance, PolyLine, Polygon, Pt2D};
use map_model::raw_data::{Hint, Hints, InitialMap, Map, StableIntersectionID, StableRoadID};
use map_model::LANE_THICKNESS;
use std::collections::HashSet;
use std::{env, process};
use viewer::World;

// Bit bigger than buses
const MIN_ROAD_LENGTH: Distance = Distance::const_meters(13.0);

struct UI {
    world: World<ID>,
    data: InitialMap,
    raw: Map,
    hints: Hints,
    state: State,
}

enum State {
    Main {
        menu: ModalMenu,
        // TODO Or, if these are common things, the World could also hold this state.
        selected: Option<ID>,
        osd: Text,
    },
    BrowsingHints(WarpingItemSlider<Hint>),
}

impl State {
    fn main(ctx: &mut EventCtx) -> State {
        State::Main {
            menu: ModalMenu::new(
                "Fix Map Geometry",
                vec![
                    (hotkey(Key::Escape), "quit"),
                    (hotkey(Key::S), "save"),
                    (hotkey(Key::R), "reset hints"),
                    (hotkey(Key::U), "undo last hint"),
                    (hotkey(Key::B), "browse hints"),
                ],
                ctx,
            ),
            selected: None,
            osd: Text::new(),
        }
    }
}

impl UI {
    fn new(filename: &str, ctx: &mut EventCtx) -> UI {
        ctx.loading_screen(&format!("load {}", filename), |ctx, mut timer| {
            let raw: Map = abstutil::read_binary(filename, &mut timer).unwrap();
            let map_name = abstutil::basename(filename);
            let gps_bounds = &raw.gps_bounds;
            let mut data = InitialMap::new(
                map_name,
                &raw,
                gps_bounds,
                &gps_bounds.to_bounds(),
                &mut timer,
            );
            let hints = Hints::load();
            data.apply_hints(&hints, &raw, &mut timer);

            let world = initial_map_to_world(&data, ctx);

            UI {
                world,
                data,
                raw,
                hints,
                state: State::main(ctx),
            }
        })
    }
}

impl GUI for UI {
    fn event(&mut self, ctx: &mut EventCtx) -> EventLoopMode {
        match self.state {
            State::Main {
                ref mut menu,
                ref mut selected,
                ref mut osd,
            } => {
                {
                    let len = self.hints.hints.len();
                    let mut txt = Text::prompt("Fix Map Geometry");
                    txt.push(format!("[cyan:{}] hints", len));
                    for i in (1..=5).rev() {
                        if len >= i {
                            txt.add_line(describe(&self.hints.hints[len - i]));
                        } else {
                            txt.add_line("...".to_string());
                        }
                    }
                    if let Some(ID::Road(r)) = selected {
                        txt.push(format!(
                            "[red:{}] is {} long",
                            r,
                            self.data.roads[&r].trimmed_center_pts.length()
                        ));
                        for (k, v) in &self.raw.roads[&r].osm_tags {
                            txt.push(format!("[cyan:{}] = [red:{}]", k, v));
                        }
                    }
                    if let Some(ID::Intersection(i)) = selected {
                        txt.push(format!("[red:{}] OSM tag diffs:", i));
                        let roads = &self.data.intersections[&i].roads;
                        if roads.len() == 2 {
                            let mut iter = roads.iter();
                            let r1_tags = &self.raw.roads[iter.next().unwrap()].osm_tags;
                            let r2_tags = &self.raw.roads[iter.next().unwrap()].osm_tags;

                            for (k, v1) in r1_tags {
                                if let Some(v2) = r2_tags.get(k) {
                                    if v1 != v2 {
                                        txt.push(format!(
                                            "[cyan:{}] = [red:{}] / [red:{}]",
                                            k, v1, v2
                                        ));
                                    }
                                } else {
                                    txt.push(format!("[cyan:{}] = [red:{}] / MISSING", k, v1));
                                }
                            }
                            for (k, v2) in r2_tags {
                                if !r1_tags.contains_key(k) {
                                    txt.push(format!("[cyan:{}] = MISSING / [red:{}] ", k, v2));
                                }
                            }
                        }
                    }
                    menu.handle_event(ctx, Some(txt));
                }
                ctx.canvas.handle_event(ctx.input);

                if ctx.redo_mouseover() {
                    *selected = self.world.mouseover_something(ctx, &HashSet::new());
                }

                if menu.action("quit") {
                    process::exit(0);
                }
                if !self.hints.hints.is_empty() {
                    if menu.action("save") {
                        abstutil::write_json("../data/hints.json", &self.hints).unwrap();
                        println!("Saved hints.json");
                    }

                    if menu.action("browse hints") {
                        self.state = State::BrowsingHints(WarpingItemSlider::new(
                            // TODO bleh
                            self.hints
                                .hints
                                .iter()
                                .filter_map(|h| {
                                    let gps_pt = match h {
                                        Hint::MergeRoad(r) | Hint::DeleteRoad(r) => {
                                            self.raw.roads[&self.raw.find_r(*r)?].points[0]
                                        }
                                        Hint::MergeDegenerateIntersection(i) => {
                                            self.raw.intersections[&self.raw.find_i(*i)?].point
                                        }
                                    };
                                    let pt = Pt2D::from_gps(gps_pt, &self.raw.gps_bounds)?;
                                    Some((pt, h.clone()))
                                })
                                .collect(),
                            "Hints Browser",
                            "hint",
                            ctx,
                        ));
                        return EventLoopMode::InputOnly;
                    }

                    let recalc = if menu.action("undo last hint") {
                        self.hints.hints.pop();
                        true
                    } else if menu.action("reset hints") {
                        self.hints.hints.clear();
                        true
                    } else {
                        false
                    };
                    if recalc {
                        *selected = None;
                        ctx.loading_screen("recalculate map from hints", |ctx, mut timer| {
                            let gps_bounds = &self.raw.gps_bounds;
                            self.data = InitialMap::new(
                                self.data.name.clone(),
                                &self.raw,
                                gps_bounds,
                                &gps_bounds.to_bounds(),
                                &mut timer,
                            );
                            self.data.apply_hints(&self.hints, &self.raw, &mut timer);
                            self.world = initial_map_to_world(&self.data, ctx);
                        });
                        return EventLoopMode::InputOnly;
                    }
                }

                if let Some(ID::Road(r)) = selected {
                    if ctx.input.key_pressed(Key::M, "merge") {
                        self.hints
                            .hints
                            .push(Hint::MergeRoad(self.raw.roads[&r].orig_id()));
                        self.data.merge_road(*r, &mut Timer::new("merge road"));
                        self.world = initial_map_to_world(&self.data, ctx);
                        *selected = None;
                    } else if ctx.input.key_pressed(Key::D, "delete") {
                        self.hints
                            .hints
                            .push(Hint::DeleteRoad(self.raw.roads[r].orig_id()));
                        self.data.delete_road(*r, &mut Timer::new("delete road"));
                        self.world = initial_map_to_world(&self.data, ctx);
                        *selected = None;
                    }
                }
                if let Some(ID::Intersection(i)) = selected {
                    if self.data.intersections[i].roads.len() == 2
                        && ctx.input.key_pressed(Key::M, "merge")
                    {
                        self.hints.hints.push(Hint::MergeDegenerateIntersection(
                            self.raw.intersections[i].orig_id(),
                        ));
                        self.data.merge_degenerate_intersection(
                            *i,
                            &mut Timer::new("merge intersection"),
                        );
                        self.world = initial_map_to_world(&self.data, ctx);
                        *selected = None;
                    }
                }

                *osd = Text::new();
                ctx.input.populate_osd(osd);
                EventLoopMode::InputOnly
            }
            State::BrowsingHints(ref mut slider) => {
                ctx.canvas.handle_event(ctx.input);
                let mut txt = Text::prompt("Hints Browser");
                {
                    let (idx, hint) = slider.get();
                    txt.add_line(format!("Hint {}/{}", idx + 1, slider.len()));
                    txt.add_line(describe(hint));
                }
                if let Some((evmode, _)) = slider.event(ctx, Some(txt)) {
                    evmode
                } else {
                    self.state = State::main(ctx);
                    EventLoopMode::InputOnly
                }
            }
        }
    }

    fn draw(&self, g: &mut GfxCtx) {
        g.clear(Color::WHITE);

        self.world.draw(g, &HashSet::new());

        match self.state {
            State::Main {
                ref selected,
                ref menu,
                ref osd,
            } => {
                if let Some(id) = selected {
                    self.world.draw_selected(g, *id);
                }

                menu.draw(g);
                g.draw_blocking_text(osd, ezgui::BOTTOM_LEFT);
            }
            State::BrowsingHints(ref slider) => {
                let poly =
                    match slider.get().1 {
                        Hint::MergeRoad(r) | Hint::DeleteRoad(r) => {
                            PolyLine::new(self.raw.gps_bounds.must_convert(
                                &self.raw.roads[&self.raw.find_r(*r).unwrap()].points,
                            ))
                            // Just make up a width
                            .make_polygons(4.0 * LANE_THICKNESS)
                        }
                        Hint::MergeDegenerateIntersection(i) => Circle::new(
                            Pt2D::from_gps(
                                self.raw.intersections[&self.raw.find_i(*i).unwrap()].point,
                                &self.raw.gps_bounds,
                            )
                            .unwrap(),
                            Distance::meters(10.0),
                        )
                        .to_polygon(),
                    };
                g.draw_polygon(Color::PURPLE.alpha(0.7), &poly);

                slider.draw(g);
            }
        }
    }
}

fn main() {
    let args: Vec<String> = env::args().collect();
    ezgui::run("InitialMap debugger", 1800.0, 800.0, |ctx| {
        ctx.canvas.cam_zoom = 4.0;
        UI::new(&args[1], ctx)
    });
}

#[derive(Debug, PartialEq, Eq, Hash, Clone, Copy)]
enum ID {
    Road(StableRoadID),
    Intersection(StableIntersectionID),
}

impl viewer::ObjectID for ID {
    fn zorder(&self) -> usize {
        match self {
            ID::Road(_) => 0,
            ID::Intersection(_) => 1,
        }
    }
}

fn initial_map_to_world(data: &InitialMap, ctx: &mut EventCtx) -> World<ID> {
    let mut w = World::new(&data.bounds);

    for r in data.roads.values() {
        w.add_obj(
            ctx.prerender,
            ID::Road(r.id),
            (if r.fwd_width >= r.back_width {
                r.trimmed_center_pts
                    .shift_right((r.fwd_width - r.back_width) / 2.0)
            } else {
                r.trimmed_center_pts
                    .shift_left((r.back_width - r.fwd_width) / 2.0)
            })
            .unwrap()
            .make_polygons(r.fwd_width + r.back_width),
            if r.trimmed_center_pts.length() < MIN_ROAD_LENGTH {
                Color::CYAN
            } else {
                Color::grey(0.8)
            },
            Text::from_line(r.id.to_string()),
        );
    }

    for i in data.intersections.values() {
        w.add_obj(
            ctx.prerender,
            ID::Intersection(i.id),
            Polygon::new(&i.polygon),
            if i.roads.len() == 2 {
                Color::RED
            } else {
                Color::BLACK
            },
            Text::from_line(format!("{}", i.id)),
        );
    }

    w
}

fn describe(hint: &Hint) -> String {
    match hint {
        Hint::MergeRoad(_) => "MergeRoad(...)".to_string(),
        Hint::DeleteRoad(_) => "DeleteRoad(...)".to_string(),
        Hint::MergeDegenerateIntersection(_) => "MergeDegenerateIntersection(...)".to_string(),
    }
}
