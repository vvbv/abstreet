use crate::{DirectedRoadID, LaneType, Map, Path, PathRequest, PathStep, RoadID, TurnID};
use abstutil::Timer;
use derivative::Derivative;
use rust_ch::{ContractionHierarchy, InputGraph};
use serde_derive::{Deserialize, Serialize};
use std::collections::{BTreeSet, HashSet, VecDeque};

// TODO Make the graph smaller by considering RoadID, or even (directed?) bundles of roads based on
// OSM way.
#[derive(Serialize, Deserialize, Derivative)]
#[derivative(Debug)]
pub struct VehiclePathfinder {
    #[derivative(Debug = "ignore")]
    ch: ContractionHierarchy,
    lane_types: Vec<LaneType>,
}

pub enum Outcome {
    Success(Path),
    Failure,
    RetrySlow,
}

impl VehiclePathfinder {
    pub fn new(map: &Map, lane_types: Vec<LaneType>, timer: &mut Timer) -> VehiclePathfinder {
        let mut g = InputGraph::new();

        timer.start("building InputGraph");
        let mut existing_edges = HashSet::new();
        for t in map.all_turns().values() {
            if !map.is_turn_allowed(t.id) {
                continue;
            }
            let src_l = map.get_l(t.id.src);
            let dst_l = map.get_l(t.id.dst);
            if !lane_types.contains(&src_l.lane_type) || !lane_types.contains(&dst_l.lane_type) {
                continue;
            }
            // First length arbitrarily wins.
            let edge = (
                src_l.get_directed_parent(map),
                dst_l.get_directed_parent(map),
            );
            if existing_edges.contains(&edge) {
                continue;
            }
            // TODO Speed limit or some other cost
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

        VehiclePathfinder { ch, lane_types }
    }

    pub fn pathfind(&self, req: &PathRequest, map: &Map) -> Outcome {
        assert!(!map.get_l(req.start.lane()).is_sidewalk());

        let path = self.ch.calc_path(
            node_idx(map.get_l(req.start.lane()).get_directed_parent(map)),
            node_idx(map.get_l(req.end.lane()).get_directed_parent(map)),
        );
        if path.get_nodes().is_empty() {
            return Outcome::Failure;
        }

        // TODO windows(2) would be fine for peeking, except it drops the last element for odd
        // cardinality
        let mut nodes = VecDeque::new();
        for node in path.get_nodes() {
            nodes.push_back(idx_to_node(*node));
        }

        let mut steps: Vec<PathStep> = Vec::new();
        while !nodes.is_empty() {
            let dr = nodes.pop_front().unwrap();
            if steps.is_empty() {
                steps.push(PathStep::Lane(req.start.lane()));
            } else {
                let from_lane = match steps.last() {
                    Some(PathStep::Lane(l)) => *l,
                    _ => unreachable!(),
                };
                if let Some(turn) = map.get_turns_from_lane(from_lane).into_iter().find(|t| {
                    // Special case the last step
                    if nodes.is_empty() {
                        t.id.dst == req.end.lane()
                    } else {
                        let l = map.get_l(t.id.dst);
                        if l.get_directed_parent(map) == dr {
                            // TODO different case when nodes.len() == 1.
                            map.get_turns_from_lane(l.id)
                                .into_iter()
                                .any(|t2| map.get_l(t2.id.dst).get_directed_parent(map) == nodes[0])
                        } else {
                            false
                        }
                    }
                }) {
                    steps.push(PathStep::Turn(turn.id));
                    steps.push(PathStep::Lane(turn.id.dst));
                } else {
                    if steps.len() == 1 {
                        // Started in the wrong lane
                        return Outcome::RetrySlow;
                    } else {
                        // Need more lookahead to stitch together the right path
                        return Outcome::RetrySlow;
                    }
                }
            }
        }
        Outcome::Success(Path::new(map, steps, req.end.dist_along()))
    }

    pub fn apply_edits(
        &mut self,
        delete_turns: &BTreeSet<TurnID>,
        add_turns: &BTreeSet<TurnID>,
        map: &Map,
        timer: &mut Timer,
    ) {
        // TODO Recalculate from scratch or something else
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
