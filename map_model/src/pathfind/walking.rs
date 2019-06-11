use crate::{
    BusRouteID, BusStopID, DirectedRoadID, IntersectionID, Lane, LaneType, Map, Path, PathRequest,
    PathStep, Position, RoadID,
};
use abstutil::Timer;
use derivative::Derivative;
use fast_paths::{ContractionHierarchy, InputGraph};
use serde_derive::{Deserialize, Serialize};
use std::collections::HashSet;

// TODO Make the graph smaller by considering RoadID, or even (directed?) bundles of roads based on
// OSM way.
#[derive(Serialize, Deserialize, Derivative)]
#[derivative(Debug)]
pub struct SidewalkPathfinder {
    #[derivative(Debug = "ignore")]
    ch: ContractionHierarchy,
}

// TODO Now lots of the code is same as VehiclePathfinder.
impl SidewalkPathfinder {
    pub fn new(map: &Map, _use_transit: bool, timer: &mut Timer) -> SidewalkPathfinder {
        let mut g = InputGraph::new();

        timer.start("building InputGraph");
        let mut existing_edges = HashSet::new();
        for t in map.all_turns().values() {
            if !t.between_sidewalks() || !map.is_turn_allowed(t.id) {
                continue;
            }
            let src_l = map.get_l(t.id.src);
            let dst_l = map.get_l(t.id.dst);
            // First length arbitrarily wins.
            let edge = (
                src_l.get_directed_parent(map),
                dst_l.get_directed_parent(map),
            );
            if existing_edges.contains(&edge) {
                continue;
            }
            let length = src_l.length() + t.geom.length();
            let length_cm = (length.inner_meters() * 100.0).round() as usize;

            g.add_edge(node_idx(edge.0), node_idx(edge.1), length_cm);
            existing_edges.insert(edge);
        }
        timer.stop("building InputGraph");

        timer.start("prepare CH");
        let mut ch = ContractionHierarchy::new(g.get_num_nodes());
        ch.prepare(&g);
        timer.stop("prepare CH");

        SidewalkPathfinder { ch }
    }

    fn get_sidewalk<'a>(&self, dr: DirectedRoadID, map: &'a Map) -> &'a Lane {
        let r = map.get_r(dr.id);
        let lanes = if dr.forwards {
            &r.children_forwards
        } else {
            &r.children_backwards
        };
        for (id, lt) in lanes {
            if *lt == LaneType::Sidewalk {
                return map.get_l(*id);
            }
        }
        panic!("{} has no sidewalk", dr);
    }

    pub fn pathfind(&self, req: &PathRequest, map: &Map) -> Option<Path> {
        // Special-case one-step paths.
        if req.start.lane() == req.end.lane() {
            assert!(req.start.dist_along() != req.end.dist_along());
            if req.start.dist_along() < req.end.dist_along() {
                return Some(Path::new(
                    map,
                    vec![PathStep::Lane(req.start.lane())],
                    req.end.dist_along(),
                ));
            } else {
                return Some(Path::new(
                    map,
                    vec![PathStep::ContraflowLane(req.start.lane())],
                    req.end.dist_along(),
                ));
            }
        }

        assert!(map.get_l(req.start.lane()).is_sidewalk());

        let path = self.ch.calc_path(
            node_idx(map.get_l(req.start.lane()).get_directed_parent(map)),
            node_idx(map.get_l(req.end.lane()).get_directed_parent(map)),
        );
        if path.get_nodes().is_empty() {
            return None;
        }

        let mut steps: Vec<PathStep> = Vec::new();
        // If the request starts at the beginning/end of a lane, still include that as the first
        // PathStep. Sim layer breaks otherwise.
        let mut current_i: Option<IntersectionID> = None;

        for pair in path.get_nodes().windows(2) {
            let lane1 = self.get_sidewalk(idx_to_node(pair[0]), map);
            let l2 = self.get_sidewalk(idx_to_node(pair[1]), map).id;

            let fwd_t = map.get_turn_between(lane1.id, l2, lane1.dst_i);
            let back_t = map.get_turn_between(lane1.id, l2, lane1.src_i);
            // TODO If both are available, we sort of need to lookahead to pick the better one.
            // Oh well.
            if fwd_t.is_some() {
                if current_i != Some(lane1.dst_i) {
                    steps.push(PathStep::Lane(lane1.id));
                }
                steps.push(PathStep::Turn(fwd_t.unwrap()));
                current_i = Some(lane1.dst_i);
            } else {
                if current_i != Some(lane1.src_i) {
                    steps.push(PathStep::ContraflowLane(lane1.id));
                }
                steps.push(PathStep::Turn(back_t.unwrap()));
                current_i = Some(lane1.src_i);
            }
        }

        // Don't end a path in a turn; sim layer breaks.
        let last_lane = self.get_sidewalk(idx_to_node(*path.get_nodes().last().unwrap()), map);
        if Some(last_lane.src_i) == current_i {
            steps.push(PathStep::Lane(last_lane.id));
        } else if Some(last_lane.dst_i) == current_i {
            steps.push(PathStep::ContraflowLane(last_lane.id));
        } else {
            unreachable!();
        }

        Some(Path::new(map, steps, req.end.dist_along()))
    }

    // Attempt the pathfinding and see if riding a bus is a step.
    pub fn should_use_transit(
        &self,
        _map: &Map,
        _start: Position,
        _end: Position,
    ) -> Option<(BusStopID, BusStopID, BusRouteID)> {
        None
    }
}

fn node_idx(id: DirectedRoadID) -> usize {
    let i = 2 * id.id.0;
    if id.forwards {
        i
    } else {
        i + 1
    }
}

fn idx_to_node(idx: usize) -> DirectedRoadID {
    let id = RoadID(idx / 2);
    if idx % 2 == 0 {
        id.forwards()
    } else {
        id.backwards()
    }
}
