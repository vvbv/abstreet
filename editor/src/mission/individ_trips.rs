use crate::common::CommonState;
use crate::helpers::ID;
use crate::mission::trips::{clip_trips, Trip, TripEndpt};
use crate::ui::{ShowEverything, UI};
use abstutil::prettyprint_usize;
use ezgui::{hotkey, Color, EventCtx, GfxCtx, ItemSlider, Key, Text};
use geom::{Circle, Distance, Line, Speed};
use map_model::BuildingID;
use popdat::psrc;
use std::collections::HashMap;

pub struct TripsVisualizer {
    slider: ItemSlider<Trip>,
    bldgs: HashMap<BuildingID, psrc::Parcel>,
}

impl TripsVisualizer {
    pub fn new(ctx: &mut EventCtx, ui: &UI) -> TripsVisualizer {
        let (trips, bldgs) = ctx.loading_screen("load trip data", |_, mut timer| {
            // TODO We'll break if there are no matching trips
            clip_trips(ui, &mut timer)
        });
        TripsVisualizer {
            slider: ItemSlider::new(
                trips,
                "Trips Visualizer",
                "trip",
                vec![(hotkey(Key::Escape), "quit")],
                ctx,
            ),
            bldgs,
        }
    }

    // Returns true if the we're done
    pub fn event(&mut self, ctx: &mut EventCtx, ui: &mut UI) -> bool {
        let (idx, trip) = self.slider.get();
        let mut txt = Text::prompt("Trips Visualizer");
        txt.add_line(format!(
            "Trip {}/{}",
            prettyprint_usize(idx + 1),
            prettyprint_usize(self.slider.len())
        ));
        txt.add_line(format!("Leave at {}", trip.depart_at));
        txt.add_line(format!(
            "Purpose: {:?} -> {:?}",
            trip.purpose.0, trip.purpose.1
        ));
        txt.add_line(format!("Mode: {:?}", trip.mode));
        txt.add_line(format!("Trip time: {}", trip.trip_time));
        txt.add_line(format!("Trip distance: {}", trip.trip_dist));
        txt.add_line(format!(
            "Average speed {}",
            Speed::from_dist_time(trip.trip_dist, trip.trip_time)
        ));

        self.slider.event(ctx, Some(txt));
        ctx.canvas.handle_event(ctx.input);

        if ctx.redo_mouseover() {
            ui.primary.current_selection = ui.recalculate_current_selection(
                ctx,
                &ui.primary.sim,
                &ShowEverything::new(),
                false,
            );
        }

        if self.slider.action("quit") {
            return true;
        }
        false
    }

    pub fn draw(&self, g: &mut GfxCtx, ui: &UI) {
        let (_, trip) = self.slider.get();
        let from = trip.from.polygon(&ui.primary.map);
        let to = trip.to.polygon(&ui.primary.map);

        g.draw_polygon(Color::RED, from);
        g.draw_polygon(Color::BLUE, to);

        // Hard to see the buildings/intersections highlighted, so also a big circle...
        g.draw_circle(
            Color::RED.alpha(0.5),
            &Circle::new(from.center(), Distance::meters(100.0)),
        );
        g.draw_circle(
            Color::BLUE.alpha(0.5),
            &Circle::new(to.center(), Distance::meters(100.0)),
        );

        // For borders, draw the original out-of-bounds points.
        match trip.from {
            TripEndpt::Border(_, pt) => g.draw_line(
                Color::RED,
                Distance::meters(25.0),
                &Line::new(pt, from.center()),
            ),
            TripEndpt::Building(_) => {}
        }
        match trip.to {
            TripEndpt::Border(_, pt) => g.draw_line(
                Color::BLUE,
                Distance::meters(25.0),
                &Line::new(pt, to.center()),
            ),
            TripEndpt::Building(_) => {}
        }

        self.slider.draw(g);
        if let Some(ID::Building(b)) = ui.primary.current_selection {
            let mut osd = Text::new();
            osd.append(format!("{}", b), Some(ui.cs.get("OSD ID color")));
            osd.append(" is ".to_string(), None);
            osd.append(
                ui.primary.map.get_b(b).get_name(),
                Some(ui.cs.get("OSD name color")),
            );
            if let Some(md) = self.bldgs.get(&b) {
                osd.append(
                    format!(
                        ". {} households, {} employees",
                        md.num_households, md.num_employees,
                    ),
                    None,
                );
            }
            CommonState::draw_custom_osd(g, osd);
        } else {
            CommonState::draw_osd(g, ui, ui.primary.current_selection);
        }
    }
}
