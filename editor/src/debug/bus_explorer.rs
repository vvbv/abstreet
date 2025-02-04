use crate::common::CommonState;
use crate::helpers::ID;
use crate::ui::{ShowEverything, UI};
use ezgui::{EventCtx, EventLoopMode, GfxCtx, Key, Text, WarpingItemSlider};
use geom::Pt2D;
use map_model::BusStopID;

pub struct BusRouteExplorer {
    slider: WarpingItemSlider<BusStopID>,
    route_name: String,
}

impl BusRouteExplorer {
    pub fn new(ctx: &mut EventCtx, ui: &UI) -> Option<BusRouteExplorer> {
        let map = &ui.primary.map;
        // TODO Pick from a menu of all possible routes
        let route = match ui.primary.current_selection {
            Some(ID::BusStop(bs)) => map.get_routes_serving_stop(bs).pop()?,
            _ => {
                return None;
            }
        };
        if !ctx.input.contextual_action(Key::E, "explore bus route") {
            return None;
        }

        let stops: Vec<(Pt2D, BusStopID)> = route
            .stops
            .iter()
            .map(|bs| {
                let stop = map.get_bs(*bs);
                (stop.sidewalk_pos.pt(map), stop.id)
            })
            .collect();

        Some(BusRouteExplorer {
            route_name: route.name.clone(),
            slider: WarpingItemSlider::new(stops, "Bus Route Explorer", "stop", ctx),
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

        let (idx, stop_id) = self.slider.get();
        let stop_id = *stop_id;
        let mut txt = Text::prompt(&format!("Bus Route Explorer for {:?}", self.route_name));
        txt.add_line(format!("Step {}/{}", idx + 1, self.slider.len()));

        let (evmode, done_warping) = self.slider.event(ctx, Some(txt))?;
        if done_warping {
            ui.primary.current_selection = Some(ID::BusStop(stop_id));
        }
        Some(evmode)
    }

    pub fn draw(&self, g: &mut GfxCtx, ui: &UI) {
        self.slider.draw(g);
        CommonState::draw_osd(g, ui, ui.primary.current_selection);
    }
}
