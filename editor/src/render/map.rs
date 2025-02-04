use crate::helpers::{ColorScheme, ID};
use crate::render::area::DrawArea;
use crate::render::building::DrawBuilding;
use crate::render::bus_stop::DrawBusStop;
use crate::render::extra_shape::{DrawExtraShape, ExtraShapeID};
use crate::render::intersection::DrawIntersection;
use crate::render::lane::DrawLane;
use crate::render::road::DrawRoad;
use crate::render::turn::DrawTurn;
use crate::render::Renderable;
use crate::ui::Flags;
use aabb_quadtree::QuadTree;
use abstutil::Timer;
use ezgui::{Color, Drawable, GeomBatch, Prerender};
use geom::{Bounds, Duration, FindClosest};
use map_model::{
    AreaID, BuildingID, BusStopID, DirectedRoadID, IntersectionID, IntersectionType, Lane, LaneID,
    Map, RoadID, Traversable, Turn, TurnID, TurnType, LANE_THICKNESS,
};
use std::borrow::Borrow;
use std::cell::RefCell;
use std::collections::HashMap;

pub struct DrawMap {
    pub roads: Vec<DrawRoad>,
    pub lanes: Vec<DrawLane>,
    pub intersections: Vec<DrawIntersection>,
    pub turns: HashMap<TurnID, DrawTurn>,
    pub buildings: Vec<DrawBuilding>,
    pub extra_shapes: Vec<DrawExtraShape>,
    pub bus_stops: HashMap<BusStopID, DrawBusStop>,
    pub areas: Vec<DrawArea>,

    // TODO Move?
    pub agents: RefCell<AgentCache>,

    pub boundary_polygon: Drawable,
    pub draw_all_thick_roads: Drawable,
    pub draw_all_unzoomed_intersections: Drawable,
    pub draw_all_buildings: Drawable,
    pub draw_all_areas: Drawable,

    quadtree: QuadTree<ID>,
}

impl DrawMap {
    pub fn new(
        map: &Map,
        flags: &Flags,
        cs: &ColorScheme,
        prerender: &Prerender,
        timer: &mut Timer,
    ) -> DrawMap {
        let mut roads: Vec<DrawRoad> = Vec::new();
        let mut all_roads = GeomBatch::new();
        timer.start_iter("make DrawRoads", map.all_roads().len());
        for r in map.all_roads() {
            timer.next();
            let draw_r = DrawRoad::new(r, cs, prerender);
            all_roads.push(
                osm_rank_to_color(cs, r.get_rank()),
                r.get_thick_polygon().get(timer),
            );
            all_roads.push(
                cs.get_def("unzoomed outline", Color::BLACK),
                draw_r.get_outline(map),
            );
            roads.push(draw_r);
        }
        let draw_all_thick_roads = prerender.upload(all_roads);

        timer.start_iter("make DrawLanes", map.all_lanes().len());
        let mut lanes: Vec<DrawLane> = Vec::new();
        for l in map.all_lanes() {
            timer.next();
            lanes.push(DrawLane::new(
                l,
                map,
                !flags.dont_draw_lane_markings,
                cs,
                prerender,
                timer,
            ));
        }

        let mut turn_to_lane_offset: HashMap<TurnID, usize> = HashMap::new();
        for l in map.all_lanes() {
            DrawMap::compute_turn_to_lane_offset(&mut turn_to_lane_offset, l, map);
        }

        timer.start_iter("make DrawTurns", map.all_turns().len());
        let mut turns: HashMap<TurnID, DrawTurn> = HashMap::new();
        for t in map.all_turns().values() {
            timer.next();
            // There's never a reason to draw these icons; the turn priority is only ever Priority,
            // since they can't conflict with anything.
            if t.turn_type != TurnType::SharedSidewalkCorner {
                turns.insert(t.id, DrawTurn::new(map, t, turn_to_lane_offset[&t.id]));
            }
        }

        let mut intersections: Vec<DrawIntersection> = Vec::new();
        let mut all_intersections = GeomBatch::new();
        timer.start_iter("make DrawIntersections", map.all_intersections().len());
        for i in map.all_intersections() {
            timer.next();
            let draw_i = DrawIntersection::new(i, map, cs, prerender, timer);
            if i.intersection_type == IntersectionType::StopSign {
                all_intersections.push(osm_rank_to_color(cs, i.get_rank(map)), i.polygon.clone());
                all_intersections.push(cs.get("unzoomed outline"), draw_i.get_outline(map));
            } else {
                all_intersections.push(
                    cs.get_def("unzoomed interesting intersection", Color::BLACK),
                    i.polygon.clone(),
                );
            }
            intersections.push(draw_i);
        }
        let draw_all_unzoomed_intersections = prerender.upload(all_intersections);

        let mut buildings: Vec<DrawBuilding> = Vec::new();
        let mut all_buildings = GeomBatch::new();
        timer.start_iter("make DrawBuildings", map.all_buildings().len());
        for b in map.all_buildings() {
            timer.next();
            buildings.push(DrawBuilding::new(b, cs, &mut all_buildings));
        }
        let draw_all_buildings = prerender.upload(all_buildings);

        let mut extra_shapes: Vec<DrawExtraShape> = Vec::new();
        if let Some(ref path) = flags.kml {
            let raw_shapes = if path.ends_with(".kml") {
                kml::load(&path, &map.get_gps_bounds(), timer)
                    .expect("Couldn't load extra KML shapes")
                    .shapes
            } else {
                let shapes: kml::ExtraShapes =
                    abstutil::read_binary(&path, timer).expect("Couldn't load ExtraShapes");
                shapes.shapes
            };

            let mut closest: FindClosest<DirectedRoadID> = FindClosest::new(&map.get_bounds());
            for r in map.all_roads().iter() {
                closest.add(
                    r.id.forwards(),
                    r.center_pts.shift_right(LANE_THICKNESS).get(timer).points(),
                );
                closest.add(
                    r.id.backwards(),
                    r.center_pts.shift_left(LANE_THICKNESS).get(timer).points(),
                );
            }

            let gps_bounds = map.get_gps_bounds();
            for s in raw_shapes.into_iter() {
                if let Some(es) =
                    DrawExtraShape::new(ExtraShapeID(extra_shapes.len()), s, gps_bounds, &closest)
                {
                    extra_shapes.push(es);
                }
            }
        }

        let mut bus_stops: HashMap<BusStopID, DrawBusStop> = HashMap::new();
        for s in map.all_bus_stops().values() {
            bus_stops.insert(s.id, DrawBusStop::new(s, map, cs, prerender));
        }

        let mut areas: Vec<DrawArea> = Vec::new();
        let mut all_areas = GeomBatch::new();
        timer.start_iter("make DrawAreas", map.all_areas().len());
        for a in map.all_areas() {
            timer.next();
            areas.push(DrawArea::new(a, cs, &mut all_areas));
        }
        let draw_all_areas = prerender.upload(all_areas);

        let boundary_polygon = prerender.upload_borrowed(vec![(
            cs.get_def("map background", Color::rgb(242, 239, 233)),
            map.get_boundary_polygon(),
        )]);

        timer.start("create quadtree");
        let mut quadtree = QuadTree::default(map.get_bounds().as_bbox());
        // TODO use iter chain if everything was boxed as a renderable...
        for obj in &roads {
            quadtree.insert_with_box(obj.get_id(), obj.get_outline(map).get_bounds().as_bbox());
        }
        for obj in &lanes {
            quadtree.insert_with_box(obj.get_id(), obj.get_outline(map).get_bounds().as_bbox());
        }
        for obj in &intersections {
            quadtree.insert_with_box(obj.get_id(), obj.get_outline(map).get_bounds().as_bbox());
        }
        for obj in &buildings {
            quadtree.insert_with_box(obj.get_id(), obj.get_outline(map).get_bounds().as_bbox());
        }
        for obj in &extra_shapes {
            quadtree.insert_with_box(obj.get_id(), obj.get_outline(map).get_bounds().as_bbox());
        }
        // Don't put BusStops in the quadtree
        for obj in &areas {
            quadtree.insert_with_box(obj.get_id(), obj.get_outline(map).get_bounds().as_bbox());
        }
        timer.stop("create quadtree");

        timer.note(format!(
            "static DrawMap consumes {} MB on the GPU",
            abstutil::prettyprint_usize(prerender.get_total_bytes_uploaded() / 1024 / 1024)
        ));

        DrawMap {
            roads,
            lanes,
            intersections,
            turns,
            buildings,
            extra_shapes,
            bus_stops,
            areas,
            boundary_polygon,
            draw_all_thick_roads,
            draw_all_unzoomed_intersections,
            draw_all_buildings,
            draw_all_areas,

            agents: RefCell::new(AgentCache {
                time: None,
                agents_per_on: HashMap::new(),
            }),

            quadtree,
        }
    }

    pub fn compute_turn_to_lane_offset(result: &mut HashMap<TurnID, usize>, l: &Lane, map: &Map) {
        // Split into two groups, based on the endpoint
        let mut pair: (Vec<&Turn>, Vec<&Turn>) = map
            .get_turns_from_lane(l.id)
            .iter()
            .filter(|t| t.turn_type != TurnType::SharedSidewalkCorner)
            .partition(|t| t.id.parent == l.dst_i);

        // Sort the turn icons by angle.
        pair.0
            .sort_by_key(|t| t.angle().normalized_degrees() as i64);
        pair.1
            .sort_by_key(|t| t.angle().normalized_degrees() as i64);

        for (idx, t) in pair.0.iter().enumerate() {
            result.insert(t.id, idx);
        }
        for (idx, t) in pair.1.iter().enumerate() {
            result.insert(t.id, idx);
        }
    }

    // The alt to these is implementing std::ops::Index, but that's way more verbose!
    pub fn get_r(&self, id: RoadID) -> &DrawRoad {
        &self.roads[id.0]
    }

    pub fn get_l(&self, id: LaneID) -> &DrawLane {
        &self.lanes[id.0]
    }

    pub fn get_i(&self, id: IntersectionID) -> &DrawIntersection {
        &self.intersections[id.0]
    }

    pub fn get_t(&self, id: TurnID) -> &DrawTurn {
        &self.turns[&id]
    }

    pub fn get_turns(&self, i: IntersectionID, map: &Map) -> Vec<&DrawTurn> {
        let mut results = Vec::new();
        for t in &map.get_i(i).turns {
            if map.get_t(*t).turn_type != TurnType::SharedSidewalkCorner {
                results.push(self.get_t(*t));
            }
        }
        results
    }

    pub fn get_b(&self, id: BuildingID) -> &DrawBuilding {
        &self.buildings[id.0]
    }

    pub fn get_es(&self, id: ExtraShapeID) -> &DrawExtraShape {
        &self.extra_shapes[id.0]
    }

    pub fn get_bs(&self, id: BusStopID) -> &DrawBusStop {
        &self.bus_stops[&id]
    }

    pub fn get_a(&self, id: AreaID) -> &DrawArea {
        &self.areas[id.0]
    }

    // Unsorted, unexpanded, raw result.
    pub fn get_matching_objects(&self, bounds: Bounds) -> Vec<ID> {
        let mut results: Vec<ID> = Vec::new();
        for &(id, _, _) in &self.quadtree.query(bounds.as_bbox()) {
            results.push(*id);
        }
        results
    }
}

// TODO Invalidate when we interactively spawn stuff elsewhere?
pub struct AgentCache {
    time: Option<Duration>,
    agents_per_on: HashMap<Traversable, Vec<Box<Renderable>>>,
}

impl AgentCache {
    pub fn has(&self, time: Duration, on: Traversable) -> bool {
        if Some(time) != self.time {
            return false;
        }
        self.agents_per_on.contains_key(&on)
    }

    // Must call has() first.
    pub fn get(&self, on: Traversable) -> Vec<&dyn Renderable> {
        self.agents_per_on[&on]
            .iter()
            .map(|obj| obj.borrow())
            .collect()
    }

    pub fn put(&mut self, time: Duration, on: Traversable, agents: Vec<Box<Renderable>>) {
        if Some(time) != self.time {
            self.agents_per_on.clear();
            self.time = Some(time);
        }

        assert!(!self.agents_per_on.contains_key(&on));
        self.agents_per_on.insert(on, agents);
    }
}

fn osm_rank_to_color(cs: &ColorScheme, rank: usize) -> Color {
    if rank >= 16 {
        cs.get_def("unzoomed highway road", Color::rgb(232, 146, 162))
    } else if rank >= 6 {
        cs.get_def("unzoomed arterial road", Color::rgb(247, 250, 191))
    } else {
        cs.get_def("unzoomed residential road", Color::WHITE)
    }
}
