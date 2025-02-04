mod stop_signs;
mod traffic_signals;

use crate::common::CommonState;
use crate::debug::DebugMode;
use crate::game::{GameState, Mode};
use crate::helpers::{ColorScheme, ID};
use crate::render::{
    DrawCtx, DrawIntersection, DrawLane, DrawMap, DrawOptions, DrawTurn, Renderable,
    MIN_ZOOM_FOR_DETAIL,
};
use crate::sandbox::SandboxMode;
use crate::ui::{PerMapUI, ShowEverything, UI};
use abstutil::Timer;
use ezgui::{
    hotkey, lctrl, Color, EventCtx, EventLoopMode, GfxCtx, Key, ModalMenu, Text, Wizard,
    WrappedWizard,
};
use map_model::{
    IntersectionID, Lane, LaneID, LaneType, Map, MapEdits, Road, RoadID, TurnID, TurnType,
};
use std::collections::{BTreeSet, HashMap};

pub enum EditMode {
    ViewingDiffs(CommonState, ModalMenu),
    Saving(Wizard),
    Loading(Wizard),
    EditingStopSign(stop_signs::StopSignEditor),
    EditingTrafficSignal(traffic_signals::TrafficSignalEditor),
    BulkEditLanes(RoadID, Wizard),
}

impl EditMode {
    pub fn new(ctx: &EventCtx, ui: &mut UI) -> EditMode {
        // TODO Warn first?
        ui.primary.reset_sim();

        EditMode::ViewingDiffs(
            CommonState::new(),
            ModalMenu::new(
                "Map Edit Mode",
                vec![
                    vec![
                        (hotkey(Key::Escape), "quit"),
                        (hotkey(Key::S), "save edits"),
                        (hotkey(Key::L), "load different edits"),
                        (lctrl(Key::S), "sandbox mode"),
                        (lctrl(Key::D), "debug mode"),
                    ],
                    CommonState::modal_menu_entries(),
                ]
                .concat(),
                ctx,
            ),
        )
    }

    pub fn event(state: &mut GameState, ctx: &mut EventCtx) -> EventLoopMode {
        match state.mode {
            Mode::Edit(EditMode::ViewingDiffs(ref mut common, ref mut menu)) => {
                // The .clone() is probably not that expensive, and it makes later code a bit
                // easier to read. :)
                let orig_edits = state.ui.primary.map.get_edits().clone();
                let mut txt = Text::prompt("Map Edit Mode");
                {
                    txt.add_line(orig_edits.edits_name.clone());
                    txt.add_line(format!("{} lanes", orig_edits.lane_overrides.len()));
                    txt.add_line(format!(
                        "{} stop signs ",
                        orig_edits.stop_sign_overrides.len()
                    ));
                    txt.add_line(format!(
                        "{} traffic signals",
                        orig_edits.traffic_signal_overrides.len()
                    ));
                    txt.add_line("Right-click a lane or intersection to start editing".to_string());
                }
                menu.handle_event(ctx, Some(txt));

                ctx.canvas.handle_event(ctx.input);

                // TODO Reset when transitioning in/out of this state? Or maybe we just don't draw
                // the effects of it. Or eventually, the Option<ID> itself will live in here
                // directly.
                // TODO Only mouseover lanes and intersections?
                if ctx.redo_mouseover() {
                    state.ui.primary.current_selection = state.ui.recalculate_current_selection(
                        ctx,
                        &state.ui.primary.sim,
                        &ShowEverything::new(),
                        false,
                    );
                }
                if let Some(evmode) = common.event(ctx, &mut state.ui, menu) {
                    return evmode;
                }

                if menu.action("quit") {
                    // TODO Warn about unsaved edits
                    state.mode = Mode::SplashScreen(Wizard::new(), None);
                    return EventLoopMode::InputOnly;
                }
                if menu.action("sandbox mode") {
                    state.mode = Mode::Sandbox(SandboxMode::new(ctx));
                    return EventLoopMode::InputOnly;
                }
                if menu.action("debug mode") {
                    state.mode = Mode::Debug(DebugMode::new(ctx, &state.ui));
                    return EventLoopMode::InputOnly;
                }

                // TODO Only if current edits are unsaved
                if menu.action("save edits") {
                    state.mode = Mode::Edit(EditMode::Saving(Wizard::new()));
                    return EventLoopMode::InputOnly;
                } else if menu.action("load different edits") {
                    state.mode = Mode::Edit(EditMode::Loading(Wizard::new()));
                    return EventLoopMode::InputOnly;
                }

                if let Some(ID::Lane(id)) = state.ui.primary.current_selection {
                    // TODO Urgh, borrow checker.
                    {
                        let lane = state.ui.primary.map.get_l(id);
                        let road = state.ui.primary.map.get_r(lane.parent);
                        if lane.lane_type != LaneType::Sidewalk {
                            if let Some(new_type) =
                                next_valid_type(road, lane, &state.ui.primary.map)
                            {
                                if ctx.input.contextual_action(
                                    Key::Space,
                                    &format!("toggle to {:?}", new_type),
                                ) {
                                    let mut new_edits = orig_edits.clone();
                                    new_edits.lane_overrides.insert(lane.id, new_type);
                                    apply_map_edits(
                                        &mut state.ui.primary,
                                        &state.ui.cs,
                                        ctx,
                                        new_edits,
                                    );
                                }
                            }
                        }
                    }
                    {
                        let lane = state.ui.primary.map.get_l(id);
                        let road = state.ui.primary.map.get_r(lane.parent);
                        if lane.lane_type != LaneType::Sidewalk {
                            for (lt, name, key) in &[
                                (LaneType::Driving, "driving", Key::D),
                                (LaneType::Parking, "parking", Key::P),
                                (LaneType::Biking, "biking", Key::B),
                                (LaneType::Bus, "bus", Key::T),
                            ] {
                                if can_change_lane_type(road, lane, *lt, &state.ui.primary.map)
                                    && ctx.input.contextual_action(
                                        *key,
                                        &format!("change to {} lane", name),
                                    )
                                {
                                    let mut new_edits = orig_edits.clone();
                                    new_edits.lane_overrides.insert(lane.id, *lt);
                                    apply_map_edits(
                                        &mut state.ui.primary,
                                        &state.ui.cs,
                                        ctx,
                                        new_edits,
                                    );
                                    break;
                                }
                            }
                        }
                    }

                    if ctx
                        .input
                        .contextual_action(Key::U, "bulk edit lanes on this road")
                    {
                        state.mode = Mode::Edit(EditMode::BulkEditLanes(
                            state.ui.primary.map.get_l(id).parent,
                            Wizard::new(),
                        ));
                    } else if orig_edits.lane_overrides.contains_key(&id)
                        && ctx.input.contextual_action(Key::R, "revert")
                    {
                        let mut new_edits = orig_edits.clone();
                        new_edits.lane_overrides.remove(&id);
                        apply_map_edits(&mut state.ui.primary, &state.ui.cs, ctx, new_edits);
                    }
                }
                if let Some(ID::Intersection(id)) = state.ui.primary.current_selection {
                    if state.ui.primary.map.maybe_get_stop_sign(id).is_some() {
                        if ctx
                            .input
                            .contextual_action(Key::E, &format!("edit stop signs for {}", id))
                        {
                            state.mode = Mode::Edit(EditMode::EditingStopSign(
                                stop_signs::StopSignEditor::new(id, ctx, &mut state.ui),
                            ));
                        } else if orig_edits.stop_sign_overrides.contains_key(&id)
                            && ctx.input.contextual_action(Key::R, "revert")
                        {
                            let mut new_edits = orig_edits.clone();
                            new_edits.stop_sign_overrides.remove(&id);
                            apply_map_edits(&mut state.ui.primary, &state.ui.cs, ctx, new_edits);
                        }
                    }
                    if state.ui.primary.map.maybe_get_traffic_signal(id).is_some() {
                        if ctx
                            .input
                            .contextual_action(Key::E, &format!("edit traffic signal for {}", id))
                        {
                            state.mode = Mode::Edit(EditMode::EditingTrafficSignal(
                                traffic_signals::TrafficSignalEditor::new(id, ctx, &mut state.ui),
                            ));
                        } else if orig_edits.traffic_signal_overrides.contains_key(&id)
                            && ctx.input.contextual_action(Key::R, "revert")
                        {
                            let mut new_edits = orig_edits.clone();
                            new_edits.traffic_signal_overrides.remove(&id);
                            apply_map_edits(&mut state.ui.primary, &state.ui.cs, ctx, new_edits);
                        }
                    }
                }
            }
            Mode::Edit(EditMode::Saving(ref mut wizard)) => {
                ctx.canvas.handle_event(ctx.input);
                if save_edits(wizard.wrap(ctx), &mut state.ui.primary.map).is_some()
                    || wizard.aborted()
                {
                    state.mode = Mode::Edit(EditMode::new(ctx, &mut state.ui));
                }
            }
            Mode::Edit(EditMode::Loading(ref mut wizard)) => {
                ctx.canvas.handle_event(ctx.input);
                if let Some(new_edits) = load_edits(
                    &state.ui.primary.map,
                    &mut wizard.wrap(ctx),
                    "Load which map edits?",
                ) {
                    apply_map_edits(&mut state.ui.primary, &state.ui.cs, ctx, new_edits);
                    state.mode = Mode::Edit(EditMode::new(ctx, &mut state.ui));
                } else if wizard.aborted() {
                    state.mode = Mode::Edit(EditMode::new(ctx, &mut state.ui));
                }
            }
            Mode::Edit(EditMode::EditingStopSign(ref mut editor)) => {
                if editor.event(ctx, &mut state.ui) {
                    state.mode = Mode::Edit(EditMode::new(ctx, &mut state.ui));
                }
            }
            Mode::Edit(EditMode::EditingTrafficSignal(ref mut editor)) => {
                if editor.event(ctx, &mut state.ui) {
                    state.mode = Mode::Edit(EditMode::new(ctx, &mut state.ui));
                }
            }
            Mode::Edit(EditMode::BulkEditLanes(r, ref mut wizard)) => {
                ctx.canvas.handle_event(ctx.input);
                if let Some(edits) = bulk_edit(r, &mut wizard.wrap(ctx), &state.ui.primary.map) {
                    apply_map_edits(&mut state.ui.primary, &state.ui.cs, ctx, edits);
                    state.mode = Mode::Edit(EditMode::new(ctx, &mut state.ui));
                } else if wizard.aborted() {
                    state.mode = Mode::Edit(EditMode::new(ctx, &mut state.ui));
                }
            }
            _ => unreachable!(),
        }

        EventLoopMode::InputOnly
    }

    pub fn draw(state: &GameState, g: &mut GfxCtx) {
        match state.mode {
            Mode::Edit(EditMode::ViewingDiffs(ref common, ref menu)) => {
                state.ui.draw(
                    g,
                    common.draw_options(&state.ui),
                    &state.ui.primary.sim,
                    &ShowEverything::new(),
                );

                // More generally we might want to show the diff between two edits, but for now,
                // just show diff relative to basemap.
                let edits = state.ui.primary.map.get_edits();

                let ctx = DrawCtx {
                    cs: &state.ui.cs,
                    map: &state.ui.primary.map,
                    draw_map: &state.ui.primary.draw_map,
                    sim: &state.ui.primary.sim,
                };
                let mut opts = DrawOptions::new();

                // TODO Similar to drawing areas with traffic or not -- would be convenient to just
                // supply a set of things to highlight and have something else take care of drawing
                // with detail or not.
                if g.canvas.cam_zoom >= MIN_ZOOM_FOR_DETAIL {
                    g.enable_hatching();

                    for l in edits.lane_overrides.keys() {
                        ctx.draw_map.get_l(*l).draw(g, &opts, &ctx);
                    }
                    for i in edits
                        .stop_sign_overrides
                        .keys()
                        .chain(edits.traffic_signal_overrides.keys())
                    {
                        ctx.draw_map.get_i(*i).draw(g, &opts, &ctx);
                    }

                    g.disable_hatching();

                    // The hatching covers up the selection outline, so redraw it.
                    match state.ui.primary.current_selection {
                        Some(ID::Lane(l)) => {
                            g.draw_polygon(
                                state.ui.cs.get("selected"),
                                &ctx.draw_map.get_l(l).get_outline(&ctx.map),
                            );
                        }
                        Some(ID::Intersection(i)) => {
                            g.draw_polygon(
                                state.ui.cs.get("selected"),
                                &ctx.draw_map.get_i(i).get_outline(&ctx.map),
                            );
                        }
                        _ => {}
                    }
                } else {
                    let color = state.ui.cs.get_def("unzoomed map diffs", Color::RED);
                    for l in edits.lane_overrides.keys() {
                        g.draw_polygon(color, &ctx.map.get_parent(*l).get_thick_polygon().unwrap());
                    }

                    for i in edits
                        .stop_sign_overrides
                        .keys()
                        .chain(edits.traffic_signal_overrides.keys())
                    {
                        opts.override_colors.insert(ID::Intersection(*i), color);
                        ctx.draw_map.get_i(*i).draw(g, &opts, &ctx);
                    }
                }

                common.draw(g, &state.ui);
                menu.draw(g);
            }
            Mode::Edit(EditMode::Saving(ref wizard))
            | Mode::Edit(EditMode::Loading(ref wizard))
            | Mode::Edit(EditMode::BulkEditLanes(_, ref wizard)) => {
                state.ui.draw(
                    g,
                    DrawOptions::new(),
                    &state.ui.primary.sim,
                    &ShowEverything::new(),
                );

                // TODO Still draw the diffs, yo
                wizard.draw(g);
            }
            Mode::Edit(EditMode::EditingStopSign(ref editor)) => {
                editor.draw(g, state);
            }
            Mode::Edit(EditMode::EditingTrafficSignal(ref editor)) => {
                editor.draw(g, state);
            }
            _ => unreachable!(),
        }
    }
}

fn save_edits(mut wizard: WrappedWizard, map: &mut Map) -> Option<()> {
    let rename = if map.get_edits().edits_name == "no_edits" {
        Some(wizard.input_string("Name these map edits")?)
    } else {
        None
    };

    // TODO Do it this weird way to avoid saving edits on every event. :P
    let save = "save edits";
    let cancel = "cancel";
    if wizard
        .choose_string("Overwrite edits?", vec![save, cancel])?
        .as_str()
        == save
    {
        if let Some(name) = rename {
            let mut edits = map.get_edits().clone();
            edits.edits_name = name;
            map.apply_edits(edits, &mut Timer::new("name map edits"));
        }
        map.get_edits().save();
    }
    Some(())
}

// For lane editing

fn next_valid_type(r: &Road, l: &Lane, map: &Map) -> Option<LaneType> {
    let mut new_type = next_type(l.lane_type);
    while new_type != l.lane_type {
        if can_change_lane_type(r, l, new_type, map) {
            return Some(new_type);
        }
        new_type = next_type(new_type);
    }
    None
}

fn next_type(lt: LaneType) -> LaneType {
    match lt {
        LaneType::Driving => LaneType::Parking,
        LaneType::Parking => LaneType::Biking,
        LaneType::Biking => LaneType::Bus,
        LaneType::Bus => LaneType::Driving,

        LaneType::Sidewalk => unreachable!(),
    }
}

fn can_change_lane_type(r: &Road, l: &Lane, lt: LaneType, map: &Map) -> bool {
    let (fwds, idx) = r.dir_and_offset(l.id);

    if l.lane_type == lt {
        return false;
    }

    // Only one parking lane per side.
    if lt == LaneType::Parking {
        let has_parking = if fwds {
            r.get_lane_types().0
        } else {
            r.get_lane_types().1
        }
        .contains(&LaneType::Parking);
        if has_parking {
            return false;
        }
    }

    // Two adjacent bike lanes is unnecessary.
    if lt == LaneType::Biking {
        let types = if fwds {
            r.get_lane_types().0
        } else {
            r.get_lane_types().1
        };
        if (idx != 0 && types[idx - 1] == LaneType::Biking)
            || types.get(idx + 1) == Some(&LaneType::Biking)
        {
            return false;
        }
    }

    // Don't let players orphan a bus stop.
    if !r.all_bus_stops(map).is_empty() && (lt == LaneType::Parking || lt == LaneType::Biking) {
        // Is this the last one?
        let mut other_bus_lane = false;
        for id in r.all_lanes() {
            if l.id != id {
                let other_lt = map.get_l(id).lane_type;
                if other_lt == LaneType::Driving || other_lt == LaneType::Bus {
                    other_bus_lane = true;
                    break;
                }
            }
        }
        if !other_bus_lane {
            return false;
        }
    }

    true
}

pub fn apply_map_edits(
    bundle: &mut PerMapUI,
    cs: &ColorScheme,
    ctx: &mut EventCtx,
    edits: MapEdits,
) {
    let mut timer = Timer::new("apply map edits");

    let (lanes_changed, turns_deleted, turns_added) = bundle.map.apply_edits(edits, &mut timer);

    for l in lanes_changed {
        bundle.draw_map.lanes[l.0] = DrawLane::new(
            bundle.map.get_l(l),
            &bundle.map,
            !bundle.current_flags.dont_draw_lane_markings,
            cs,
            ctx.prerender,
            &mut timer,
        );
    }
    let mut modified_intersections: BTreeSet<IntersectionID> = BTreeSet::new();
    let mut lanes_of_modified_turns: BTreeSet<LaneID> = BTreeSet::new();
    for t in turns_deleted {
        bundle.draw_map.turns.remove(&t);
        lanes_of_modified_turns.insert(t.src);
        modified_intersections.insert(t.parent);
    }
    for t in &turns_added {
        lanes_of_modified_turns.insert(t.src);
        modified_intersections.insert(t.parent);
    }

    let mut turn_to_lane_offset: HashMap<TurnID, usize> = HashMap::new();
    for l in lanes_of_modified_turns {
        DrawMap::compute_turn_to_lane_offset(
            &mut turn_to_lane_offset,
            bundle.map.get_l(l),
            &bundle.map,
        );
    }
    for t in turns_added {
        let turn = bundle.map.get_t(t);
        if turn.turn_type != TurnType::SharedSidewalkCorner {
            bundle
                .draw_map
                .turns
                .insert(t, DrawTurn::new(&bundle.map, turn, turn_to_lane_offset[&t]));
        }
    }

    for i in modified_intersections {
        bundle.draw_map.intersections[i.0] = DrawIntersection::new(
            bundle.map.get_i(i),
            &bundle.map,
            cs,
            ctx.prerender,
            &mut timer,
        );
    }

    // Do this after fixing up all the state above.
    bundle.map.simplify_edits(&mut timer);
}

fn load_edits(map: &Map, wizard: &mut WrappedWizard, query: &str) -> Option<MapEdits> {
    // TODO Exclude current?
    let map_name = map.get_name().to_string();
    wizard
        .choose_something_no_keys::<MapEdits>(
            query,
            Box::new(move || {
                let mut list = abstutil::load_all_objects("edits", &map_name);
                list.push(("no_edits".to_string(), MapEdits::new(map_name.clone())));
                list
            }),
        )
        .map(|(_, e)| e)
}

fn bulk_edit(r: RoadID, wizard: &mut WrappedWizard, map: &Map) -> Option<MapEdits> {
    let from = wizard
        .choose_something(
            "Change all lanes of type...",
            Box::new(|| {
                vec![
                    (None, "driving".to_string(), LaneType::Driving),
                    (None, "parking".to_string(), LaneType::Parking),
                    (None, "biking".to_string(), LaneType::Biking),
                    (None, "bus".to_string(), LaneType::Bus),
                ]
            }),
        )?
        .1;
    let to = wizard
        .choose_something(
            "Change to all lanes of type...",
            Box::new(move || {
                vec![
                    (None, "driving".to_string(), LaneType::Driving),
                    (None, "parking".to_string(), LaneType::Parking),
                    (None, "biking".to_string(), LaneType::Biking),
                    (None, "bus".to_string(), LaneType::Bus),
                ]
                .into_iter()
                .filter(|(_, _, lt)| *lt != from)
                .collect()
            }),
        )?
        .1;

    // Do the dirty deed. Match by road name; OSM way ID changes a fair bit.
    let road_name = map.get_r(r).get_name();
    let mut edits = map.get_edits().clone();
    let mut cnt = 0;
    for l in map.all_lanes() {
        if l.lane_type != from {
            continue;
        }
        let parent = map.get_parent(l.id);
        if parent.get_name() != road_name {
            continue;
        }
        // TODO This looks at the original state of the map, not with all the edits applied so far!
        if can_change_lane_type(parent, l, to, map) {
            edits.lane_overrides.insert(l.id, to);
            cnt += 1;
        }
    }
    // TODO pop this up. warn about road names changing and being weird. :)
    println!(
        "Changed {} {:?} lanes to {:?} lanes on {}",
        cnt, from, to, road_name
    );
    Some(edits)
}
