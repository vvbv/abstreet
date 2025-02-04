use crate::game::Mode;
use crate::mission::{input_time, MissionEditMode};
use crate::sandbox::SandboxMode;
use crate::ui::UI;
use abstutil::{Timer, WeightedUsizeChoice};
use ezgui::{hotkey, EventCtx, GfxCtx, Key, LogScroller, ModalMenu, Wizard, WrappedWizard};
use geom::Duration;
use map_model::{IntersectionID, Map, Neighborhood};
use sim::{BorderSpawnOverTime, OriginDestination, Scenario, SeedParkedCars, SpawnOverTime};

pub enum ScenarioEditor {
    PickScenario(Wizard),
    ManageScenario(ModalMenu, Scenario, LogScroller),
    EditScenario(Scenario, Wizard),
}

impl ScenarioEditor {
    fn modal_menu(name: &str, ctx: &EventCtx) -> ModalMenu {
        ModalMenu::new(
            &format!("Scenario Editor for {}", name),
            vec![
                (hotkey(Key::Escape), "quit"),
                (hotkey(Key::S), "save"),
                (hotkey(Key::E), "edit"),
                (hotkey(Key::I), "instantiate"),
            ],
            ctx,
        )
    }

    pub fn event(&mut self, ctx: &mut EventCtx, ui: &mut UI) -> Option<Mode> {
        match self {
            ScenarioEditor::PickScenario(ref mut wizard) => {
                if let Some(scenario) = pick_scenario(&ui.primary.map, wizard.wrap(ctx)) {
                    let scroller =
                        LogScroller::new(scenario.scenario_name.clone(), scenario.describe());
                    *self = ScenarioEditor::ManageScenario(
                        ScenarioEditor::modal_menu(&scenario.scenario_name, ctx),
                        scenario,
                        scroller,
                    );
                } else if wizard.aborted() {
                    return Some(Mode::Mission(MissionEditMode::new(ctx, ui)));
                }
            }
            ScenarioEditor::ManageScenario(ref mut menu, scenario, ref mut scroller) => {
                menu.handle_event(ctx, None);
                ctx.canvas.handle_event(ctx.input);
                if menu.action("save") {
                    scenario.save();
                } else if menu.action("edit") {
                    *self = ScenarioEditor::EditScenario(scenario.clone(), Wizard::new());
                } else if menu.action("instantiate") {
                    ctx.loading_screen("instantiate scenario", |_, timer| {
                        scenario.instantiate(
                            &mut ui.primary.sim,
                            &ui.primary.map,
                            &mut ui.primary.current_flags.sim_flags.make_rng(),
                            timer,
                        );
                        ui.primary.sim.step(&ui.primary.map, Duration::seconds(0.1));
                    });
                    return Some(Mode::Sandbox(SandboxMode::new(ctx)));
                } else if scroller.event(&mut ctx.input) {
                    return Some(Mode::Mission(MissionEditMode::new(ctx, ui)));
                }
            }
            ScenarioEditor::EditScenario(ref mut scenario, ref mut wizard) => {
                if let Some(()) = edit_scenario(&ui.primary.map, scenario, wizard.wrap(ctx)) {
                    let scroller =
                        LogScroller::new(scenario.scenario_name.clone(), scenario.describe());
                    // TODO autosave, or at least make it clear there are unsaved edits
                    *self = ScenarioEditor::ManageScenario(
                        ScenarioEditor::modal_menu(&scenario.scenario_name, ctx),
                        scenario.clone(),
                        scroller,
                    );
                } else if wizard.aborted() {
                    let scroller =
                        LogScroller::new(scenario.scenario_name.clone(), scenario.describe());
                    *self = ScenarioEditor::ManageScenario(
                        ScenarioEditor::modal_menu(&scenario.scenario_name, ctx),
                        scenario.clone(),
                        scroller,
                    );
                }
            }
        }
        None
    }

    pub fn draw(&self, g: &mut GfxCtx, ui: &UI) {
        match self {
            ScenarioEditor::PickScenario(wizard) => {
                wizard.draw(g);
            }
            ScenarioEditor::ManageScenario(ref menu, _, scroller) => {
                scroller.draw(g);
                menu.draw(g);
            }
            ScenarioEditor::EditScenario(_, wizard) => {
                if let Some(neighborhood) = wizard.current_menu_choice::<Neighborhood>() {
                    g.draw_polygon(ui.cs.get("neighborhood polygon"), &neighborhood.polygon);
                }
                wizard.draw(g);
            }
        }
    }
}

fn pick_scenario(map: &Map, mut wizard: WrappedWizard) -> Option<Scenario> {
    let load_existing = "Load existing scenario";
    let create_new = "Create new scenario";
    if wizard.choose_string("What scenario to edit?", vec![load_existing, create_new])?
        == load_existing
    {
        load_scenario(map, &mut wizard, "Load which scenario?")
    } else {
        let scenario_name = wizard.input_string("Name the scenario")?;
        Some(Scenario {
            scenario_name,
            map_name: map.get_name().to_string(),
            seed_parked_cars: Vec::new(),
            spawn_over_time: Vec::new(),
            border_spawn_over_time: Vec::new(),
            individ_trips: Vec::new(),
        })
    }
}

fn edit_scenario(map: &Map, scenario: &mut Scenario, mut wizard: WrappedWizard) -> Option<()> {
    let seed_parked = "Seed parked cars";
    let spawn = "Spawn agents";
    let spawn_border = "Spawn agents from a border";
    let randomize = "Randomly spawn stuff from/to every neighborhood";
    match wizard
        .choose_string(
            "What kind of edit?",
            vec![seed_parked, spawn, spawn_border, randomize],
        )?
        .as_str()
    {
        x if x == seed_parked => {
            scenario.seed_parked_cars.push(SeedParkedCars {
                neighborhood: choose_neighborhood(
                    map,
                    &mut wizard,
                    "Seed parked cars in what area?",
                )?,
                cars_per_building: input_weighted_usize(
                    &mut wizard,
                    "How many cars per building? (ex: 4,4,2)",
                )?,
            });
        }
        x if x == spawn => {
            scenario.spawn_over_time.push(SpawnOverTime {
                num_agents: wizard.input_usize("Spawn how many agents?")?,
                start_time: input_time(&mut wizard, "Start spawning when?")?,
                // TODO input interval, or otherwise enforce stop_time > start_time
                stop_time: input_time(&mut wizard, "Stop spawning when?")?,
                start_from_neighborhood: choose_neighborhood(
                    map,
                    &mut wizard,
                    "Where should the agents start?",
                )?,
                goal: choose_origin_destination(map, &mut wizard, "Where should the agents go?")?,
                percent_biking: wizard
                    .input_percent("What percent of the walking trips will bike instead?")?,
                percent_use_transit: wizard.input_percent(
                    "What percent of the walking trips will consider taking transit?",
                )?,
            });
        }
        x if x == spawn_border => {
            scenario.border_spawn_over_time.push(BorderSpawnOverTime {
                num_peds: wizard.input_usize("Spawn how many pedestrians?")?,
                num_cars: wizard.input_usize("Spawn how many cars?")?,
                num_bikes: wizard.input_usize("Spawn how many bikes?")?,
                start_time: input_time(&mut wizard, "Start spawning when?")?,
                // TODO input interval, or otherwise enforce stop_time > start_time
                stop_time: input_time(&mut wizard, "Stop spawning when?")?,
                // TODO validate it's a border!
                start_from_border: choose_intersection(
                    &mut wizard,
                    "Which border should the agents spawn at?",
                )?,
                goal: choose_origin_destination(map, &mut wizard, "Where should the agents go?")?,
                percent_use_transit: wizard.input_percent(
                    "What percent of the walking trips will consider taking transit?",
                )?,
            });
        }
        x if x == randomize => {
            let neighborhoods = Neighborhood::load_all(map.get_name(), &map.get_gps_bounds());
            for (src, _) in &neighborhoods {
                for (dst, _) in &neighborhoods {
                    scenario.spawn_over_time.push(SpawnOverTime {
                        num_agents: 100,
                        start_time: Duration::ZERO,
                        stop_time: Duration::minutes(10),
                        start_from_neighborhood: src.to_string(),
                        goal: OriginDestination::Neighborhood(dst.to_string()),
                        percent_biking: 0.1,
                        percent_use_transit: 0.2,
                    });
                }
            }
        }
        _ => unreachable!(),
    };
    Some(())
}

fn choose_neighborhood(map: &Map, wizard: &mut WrappedWizard, query: &str) -> Option<String> {
    let map_name = map.get_name().to_string();
    let gps_bounds = map.get_gps_bounds().clone();
    // Load the full object, since we usually visualize the neighborhood when menuing over it
    wizard
        .choose_something_no_keys::<Neighborhood>(
            query,
            Box::new(move || Neighborhood::load_all(&map_name, &gps_bounds)),
        )
        .map(|(n, _)| n)
}

fn load_scenario(map: &Map, wizard: &mut WrappedWizard, query: &str) -> Option<Scenario> {
    let map_name = map.get_name().to_string();
    wizard
        .choose_something_no_keys::<String>(
            query,
            Box::new(move || abstutil::list_all_objects("scenarios", &map_name)),
        )
        .map(|(_, s)| {
            abstutil::read_binary(
                &format!("../data/scenarios/{}/{}.bin", map.get_name(), s),
                &mut Timer::throwaway(),
            )
            .unwrap()
        })
}

fn input_weighted_usize(wizard: &mut WrappedWizard, query: &str) -> Option<WeightedUsizeChoice> {
    wizard.input_something(
        query,
        None,
        Box::new(|line| WeightedUsizeChoice::parse(&line)),
    )
}

// TODO Validate the intersection exists? Let them pick it with the cursor?
fn choose_intersection(wizard: &mut WrappedWizard, query: &str) -> Option<IntersectionID> {
    wizard.input_something(
        query,
        None,
        Box::new(|line| usize::from_str_radix(&line, 10).ok().map(IntersectionID)),
    )
}

fn choose_origin_destination(
    map: &Map,
    wizard: &mut WrappedWizard,
    query: &str,
) -> Option<OriginDestination> {
    let neighborhood = "Neighborhood";
    let border = "Border intersection";
    if wizard.choose_string(query, vec![neighborhood, border])? == neighborhood {
        choose_neighborhood(map, wizard, query).map(OriginDestination::Neighborhood)
    } else {
        choose_intersection(wizard, query).map(OriginDestination::Border)
    }
}
