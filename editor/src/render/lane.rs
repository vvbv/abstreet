use crate::helpers::{ColorScheme, ID};
use crate::render::{DrawCtx, DrawOptions, Renderable, OUTLINE_THICKNESS};
use abstutil::Timer;
use ezgui::{Color, Drawable, GeomBatch, GfxCtx, Prerender};
use geom::{Circle, Distance, Line, PolyLine, Polygon, Pt2D};
use map_model::{Lane, LaneID, LaneType, Map, Road, TurnType, LANE_THICKNESS, PARKING_SPOT_LENGTH};

pub struct DrawLane {
    pub id: LaneID,
    pub polygon: Polygon,
    zorder: isize,

    draw_default: Drawable,
}

impl DrawLane {
    pub fn new(
        lane: &Lane,
        map: &Map,
        draw_lane_markings: bool,
        cs: &ColorScheme,
        prerender: &Prerender,
        timer: &mut Timer,
    ) -> DrawLane {
        let road = map.get_r(lane.parent);
        let polygon = lane.lane_center_pts.make_polygons(LANE_THICKNESS);

        let mut draw = GeomBatch::new();
        draw.push(
            match lane.lane_type {
                LaneType::Driving => cs.get_def("driving lane", Color::BLACK),
                LaneType::Bus => cs.get_def("bus lane", Color::rgb(190, 74, 76)),
                LaneType::Parking => cs.get_def("parking lane", Color::grey(0.2)),
                LaneType::Sidewalk => cs.get_def("sidewalk", Color::grey(0.8)),
                LaneType::Biking => cs.get_def("bike lane", Color::rgb(15, 125, 75)),
            },
            polygon.clone(),
        );
        if draw_lane_markings {
            match lane.lane_type {
                LaneType::Sidewalk => {
                    draw.extend(
                        cs.get_def("sidewalk lines", Color::grey(0.7)),
                        calculate_sidewalk_lines(lane),
                    );
                }
                LaneType::Parking => {
                    draw.extend(
                        cs.get_def("parking lines", Color::WHITE),
                        calculate_parking_lines(lane),
                    );
                }
                LaneType::Driving | LaneType::Bus => {
                    draw.extend(
                        cs.get_def("dashed lane line", Color::WHITE),
                        calculate_driving_lines(lane, road, timer),
                    );
                    draw.extend(
                        cs.get_def("turn restrictions on lane", Color::WHITE),
                        calculate_turn_markings(map, lane, timer),
                    );
                }
                LaneType::Biking => {}
            };
            /*if lane.lane_type.is_for_moving_vehicles()
                && map.get_i(lane.dst_i).intersection_type == IntersectionType::StopSign
            {
                draw.extend(calculate_stop_sign_line(road, lane, map, cs));
            }*/
        }

        DrawLane {
            id: lane.id,
            polygon,
            zorder: road.get_zorder(),
            draw_default: prerender.upload(draw),
        }
    }

    fn draw_debug(&self, g: &mut GfxCtx, ctx: &DrawCtx) {
        let circle_color = ctx
            .cs
            .get_def("debug line endpoint", Color::rgb_f(0.8, 0.1, 0.1));

        for l in ctx.map.get_l(self.id).lane_center_pts.lines() {
            g.draw_line(
                ctx.cs.get_def("debug line", Color::RED),
                Distance::meters(0.25),
                &l,
            );
            g.draw_circle(circle_color, &Circle::new(l.pt1(), Distance::meters(0.4)));
            g.draw_circle(circle_color, &Circle::new(l.pt2(), Distance::meters(0.8)));
        }
    }
}

impl Renderable for DrawLane {
    fn get_id(&self) -> ID {
        ID::Lane(self.id)
    }

    fn draw(&self, g: &mut GfxCtx, opts: &DrawOptions, ctx: &DrawCtx) {
        if let Some(color) = opts.color(self.get_id()) {
            g.draw_polygon(color, &self.polygon);
        } else {
            g.redraw(&self.draw_default);
        }

        if opts.geom_debug_mode {
            self.draw_debug(g, ctx);
        }
    }

    fn get_outline(&self, map: &Map) -> Polygon {
        map.get_l(self.id)
            .lane_center_pts
            .to_thick_boundary(LANE_THICKNESS, OUTLINE_THICKNESS)
            .unwrap_or_else(|| self.polygon.clone())
    }

    fn contains_pt(&self, pt: Pt2D, _: &Map) -> bool {
        self.polygon.contains_pt(pt)
    }

    fn get_zorder(&self) -> isize {
        self.zorder
    }
}

// TODO this always does it at pt1
fn perp_line(l: Line, length: Distance) -> Line {
    let pt1 = l.shift_right(length / 2.0).pt1();
    let pt2 = l.shift_left(length / 2.0).pt1();
    Line::new(pt1, pt2)
}

fn calculate_sidewalk_lines(lane: &Lane) -> Vec<Polygon> {
    let tile_every = LANE_THICKNESS;

    let length = lane.length();

    let mut result = Vec::new();
    // Start away from the intersections
    let mut dist_along = tile_every;
    while dist_along < length - tile_every {
        let (pt, angle) = lane.dist_along(dist_along);
        // Reuse perp_line. Project away an arbitrary amount
        let pt2 = pt.project_away(Distance::meters(1.0), angle);
        result.push(
            perp_line(Line::new(pt, pt2), LANE_THICKNESS).make_polygons(Distance::meters(0.25)),
        );
        dist_along += tile_every;
    }

    result
}

fn calculate_parking_lines(lane: &Lane) -> Vec<Polygon> {
    // meters, but the dims get annoying below to remove
    let leg_length = Distance::meters(1.0);

    let mut result = Vec::new();
    let num_spots = lane.number_parking_spots();
    if num_spots > 0 {
        for idx in 0..=num_spots {
            let (pt, lane_angle) = lane.dist_along(PARKING_SPOT_LENGTH * (1.0 + idx as f64));
            let perp_angle = lane_angle.rotate_degs(270.0);
            // Find the outside of the lane. Actually, shift inside a little bit, since the line will
            // have thickness, but shouldn't really intersect the adjacent line when drawn.
            let t_pt = pt.project_away(LANE_THICKNESS * 0.4, perp_angle);
            // The perp leg
            let p1 = t_pt.project_away(leg_length, perp_angle.opposite());
            result.push(Line::new(t_pt, p1).make_polygons(Distance::meters(0.25)));
            // Upper leg
            let p2 = t_pt.project_away(leg_length, lane_angle);
            result.push(Line::new(t_pt, p2).make_polygons(Distance::meters(0.25)));
            // Lower leg
            let p3 = t_pt.project_away(leg_length, lane_angle.opposite());
            result.push(Line::new(t_pt, p3).make_polygons(Distance::meters(0.25)));
        }
    }

    result
}

fn calculate_driving_lines(lane: &Lane, parent: &Road, timer: &mut Timer) -> Vec<Polygon> {
    // The leftmost lanes don't have dashed white lines.
    if parent.dir_and_offset(lane.id).1 == 0 {
        return Vec::new();
    }

    let dash_separation = Distance::meters(1.5);
    let dash_len = Distance::meters(1.0);

    let lane_edge_pts = lane
        .lane_center_pts
        .shift_left(LANE_THICKNESS / 2.0)
        .get(timer);
    if lane_edge_pts.length() < dash_separation * 2.0 {
        return Vec::new();
    }
    // Don't draw the dashes too close to the ends.
    lane_edge_pts
        .exact_slice(dash_separation, lane_edge_pts.length() - dash_separation)
        .dashed_polygons(Distance::meters(0.25), dash_len, dash_separation)
}

fn calculate_turn_markings(map: &Map, lane: &Lane, timer: &mut Timer) -> Vec<Polygon> {
    let mut results = Vec::new();

    // Are there multiple driving lanes on this side of the road?
    if map
        .find_closest_lane(lane.id, vec![LaneType::Driving])
        .is_err()
    {
        return results;
    }
    if lane.length() < Distance::meters(7.0) {
        return results;
    }

    let thickness = Distance::meters(0.2);

    let common_base = lane.lane_center_pts.exact_slice(
        lane.length() - Distance::meters(7.0),
        lane.length() - Distance::meters(5.0),
    );
    results.push(common_base.make_polygons(thickness));

    // TODO Maybe draw arrows per target road, not lane
    for turn in map.get_turns_from_lane(lane.id) {
        if turn.turn_type == TurnType::LaneChangeLeft || turn.turn_type == TurnType::LaneChangeRight
        {
            continue;
        }
        results.push(
            PolyLine::new(vec![
                common_base.last_pt(),
                common_base
                    .last_pt()
                    .project_away(LANE_THICKNESS / 2.0, turn.angle()),
            ])
            .make_arrow(thickness)
            .with_context(timer, format!("turn_markings for {}", turn.id)),
        );
    }

    // Just lane-changing turns after all (common base + 2 for the arrow)
    if results.len() == 3 {
        return Vec::new();
    }
    results
}
