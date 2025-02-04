use crate::common::CommonState;
use crate::render::DrawTurn;
use crate::ui::{ShowEverything, UI};
use ezgui::{Color, EventCtx, EventLoopMode, GfxCtx, Key, Text, WarpingItemSlider};
use geom::{Distance, Polygon, Pt2D};
use map_model::{Traversable, LANE_THICKNESS};
use sim::AgentID;

pub struct RouteExplorer {
    slider: WarpingItemSlider<Traversable>,
    agent: AgentID,
    entire_trace: Option<Polygon>,
}

impl RouteExplorer {
    pub fn new(ctx: &mut EventCtx, ui: &UI) -> Option<RouteExplorer> {
        let (agent, path) = if true {
            let agent = ui.primary.current_selection.and_then(|id| id.agent_id())?;
            (agent, ui.primary.sim.get_path(agent)?.clone())
        } else {
            use map_model::{LaneID, PathRequest, Position};

            // TODO Temporary for debugging
            let agent = AgentID::Pedestrian(sim::PedestrianID(42));
            let path = ui.primary.map.pathfind(PathRequest {
                start: Position::new(LaneID(4409), Distance::meters(146.9885)),
                end: Position::new(LaneID(8188), Distance::meters(82.4241)),
                can_use_bike_lanes: false,
                can_use_bus_lanes: false,
            });
            (agent, path?)
        };

        if !ctx.input.contextual_action(Key::E, "explore route") {
            return None;
        }

        // TODO Actual start dist
        let entire_trace = path
            .trace(&ui.primary.map, Distance::ZERO, None)
            .map(|pl| pl.make_polygons(LANE_THICKNESS));

        let steps: Vec<(Pt2D, Traversable)> = path
            .get_steps()
            .iter()
            .map(|step| {
                let t = step.as_traversable();
                (
                    t.dist_along(t.length(&ui.primary.map) / 2.0, &ui.primary.map)
                        .0,
                    t,
                )
            })
            .collect();
        Some(RouteExplorer {
            agent,
            slider: WarpingItemSlider::new(steps, "Route Explorer", "step", ctx),
            entire_trace,
        })
    }

    // Done when None
    pub fn event(&mut self, ctx: &mut EventCtx, ui: &mut UI) -> Option<EventLoopMode> {
        if ctx.redo_mouseover() {
            ui.primary.current_selection = ui.recalculate_current_selection(
                ctx,
                &ui.primary.sim,
                // TODO Or use what debug mode is showing?
                &ShowEverything::new(),
                false,
            );
        }
        ctx.canvas.handle_event(ctx.input);

        let (idx, step) = self.slider.get();
        let step = *step;
        let mut txt = Text::prompt(&format!("Route Explorer for {:?}", self.agent));
        txt.add_line(format!("Step {}/{}", idx + 1, self.slider.len()));
        txt.add_line(format!("{:?}", step));

        // We don't really care about setting current_selection to the current step; drawing covers
        // it up anyway.
        self.slider.event(ctx, Some(txt)).map(|(evmode, _)| evmode)
    }

    pub fn draw(&self, g: &mut GfxCtx, ui: &UI) {
        if let Some(ref poly) = self.entire_trace {
            g.draw_polygon(ui.cs.get_def("entire route", Color::BLUE.alpha(0.2)), poly);
        }

        let color = ui.cs.get_def("current step", Color::RED);
        match self.slider.get().1 {
            Traversable::Lane(l) => {
                g.draw_polygon(color, &ui.primary.draw_map.get_l(*l).polygon);
            }
            Traversable::Turn(t) => {
                DrawTurn::draw_full(ui.primary.map.get_t(*t), g, color);
            }
        }
        self.slider.draw(g);
        CommonState::draw_osd(g, ui, ui.primary.current_selection);
    }
}
