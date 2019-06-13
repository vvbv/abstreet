use crate::{
    BusRouteID, BusStopID, DirectedRoadID, IntersectionID, LaneID, LaneType, Map, Path,
    PathRequest, PathStep, Position,
};
use abstutil::{deserialize_btreemap, serialize_btreemap, Timer};
use geom::Distance;
use petgraph::graph::{Graph, NodeIndex};
use serde_derive::{Deserialize, Serialize};
use std::collections::BTreeMap;

// TODO Make the graph smaller by considering RoadID, or even (directed?) bundles of roads based on
// OSM way.
#[derive(Serialize, Deserialize, Debug)]
pub struct SidewalkPathfinder {
    graph: Graph<DirectedRoadID, Edge>,
    #[serde(
        serialize_with = "serialize_btreemap",
        deserialize_with = "deserialize_btreemap"
    )]
    nodes: BTreeMap<DirectedRoadID, NodeIndex<u32>>,
}

#[derive(Serialize, Deserialize, Debug)]
enum Edge {
    Cross(Distance),
    RideBus(BusStopID, BusStopID, BusRouteID),
}

impl SidewalkPathfinder {
    pub fn new(map: &Map, use_transit: bool, timer: &mut Timer) -> SidewalkPathfinder {
        let mut g = SidewalkPathfinder {
            graph: Graph::new(),
            nodes: BTreeMap::new(),
        };

        for r in map.all_roads() {
            // TODO Technically, only if there's a sidewalk
            if !r.children_forwards.is_empty() {
                let id = r.id.forwards();
                g.nodes.insert(id, g.graph.add_node(id));
            }
            if !r.children_backwards.is_empty() {
                let id = r.id.backwards();
                g.nodes.insert(id, g.graph.add_node(id));
            }
        }

        for t in map.all_turns().values() {
            if !t.between_sidewalks() || !map.is_turn_allowed(t.id) {
                continue;
            }
            let src_l = map.get_l(t.id.src);
            let src = g.get_node(t.id.src, map);
            let dst = g.get_node(t.id.dst, map);
            // First length arbitrarily wins.
            if !g.graph.contains_edge(src, dst) {
                g.graph
                    .add_edge(src, dst, Edge::Cross(src_l.length() + t.geom.length()));
            }
        }

        // Add edges for all the bus rides. No transfers.
        if use_transit {
            for stop1 in map.all_bus_stops().values() {
                let src = g.get_node(stop1.sidewalk_pos.lane(), map);
                for (stop2, route) in map.get_connected_bus_stops(stop1.id).into_iter() {
                    let dst = g.get_node(map.get_bs(stop2).sidewalk_pos.lane(), map);
                    g.graph
                        .add_edge(src, dst, Edge::RideBus(stop1.id, stop2, route));
                }
            }
        }

        /*println!(
            "{} nodes, {} edges",
            g.graph.node_count(),
            g.graph.edge_count()
        );*/

        g
    }

    fn get_node(&self, lane: LaneID, map: &Map) -> NodeIndex<u32> {
        self.nodes[&map.get_l(lane).get_directed_parent(map)]
    }

    fn get_sidewalk(&self, dr: DirectedRoadID, map: &Map) -> LaneID {
        let r = map.get_r(dr.id);
        let lanes = if dr.forwards {
            &r.children_forwards
        } else {
            &r.children_backwards
        };
        for (id, lt) in lanes {
            if *lt == LaneType::Sidewalk {
                return *id;
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

        let start_node = self.get_node(req.start.lane(), map);
        let end_node = self.get_node(req.end.lane(), map);
        let end_pt = map.get_l(req.end.lane()).first_pt();

        let (_, raw_nodes) = petgraph::algo::astar(
            &self.graph,
            start_node,
            |n| n == end_node,
            |e| match e.weight() {
                Edge::Cross(dist) => *dist,
                // Free for now
                Edge::RideBus(_, _, _) => Distance::ZERO,
            },
            |n| {
                let dr = self.graph[n];
                let r = map.get_r(dr.id);
                if dr.forwards {
                    end_pt.dist_to(r.center_pts.last_pt())
                } else {
                    end_pt.dist_to(r.center_pts.first_pt())
                }
            },
        )?;

        let mut steps: Vec<PathStep> = Vec::new();
        // If the request starts at the beginning/end of a lane, still include that as the first
        // PathStep. Sim layer breaks otherwise.
        let mut current_i: Option<IntersectionID> = None;

        for pair in raw_nodes.windows(2) {
            let lane1 = map.get_l(self.get_sidewalk(self.graph[pair[0]], map));
            let l2 = self.get_sidewalk(self.graph[pair[1]], map);

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
        let last_lane = map.get_l(self.get_sidewalk(self.graph[*raw_nodes.last().unwrap()], map));
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
        map: &Map,
        start: Position,
        end: Position,
    ) -> Option<(BusStopID, BusStopID, BusRouteID)> {
        let start_node = self.get_node(start.lane(), map);
        let end_node = self.get_node(end.lane(), map);
        let end_pt = map.get_l(end.lane()).first_pt();

        let (_, raw_nodes) = petgraph::algo::astar(
            &self.graph,
            start_node,
            |n| n == end_node,
            |e| match e.weight() {
                Edge::Cross(dist) => *dist,
                // Free for now
                Edge::RideBus(_, _, _) => Distance::ZERO,
            },
            |n| {
                let dr = self.graph[n];
                let r = map.get_r(dr.id);
                if dr.forwards {
                    end_pt.dist_to(r.center_pts.last_pt())
                } else {
                    end_pt.dist_to(r.center_pts.first_pt())
                }
            },
        )?;

        for pair in raw_nodes.windows(2) {
            if let Edge::RideBus(stop1, stop2, route) =
                self.graph[self.graph.find_edge(pair[0], pair[1]).unwrap()]
            {
                return Some((stop1, stop2, route));
            }
        }
        None
    }
}
