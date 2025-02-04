use crate::{raw_data, LaneID, LaneType, Map, Road, RoadID, TurnID};
use abstutil;
use geom::Polygon;
use serde_derive::{Deserialize, Serialize};
use std::collections::BTreeSet;
use std::fmt;

// TODO reconsider pub usize. maybe outside world shouldnt know.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq, PartialOrd, Ord, Serialize, Deserialize)]
pub struct IntersectionID(pub usize);

impl fmt::Display for IntersectionID {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "IntersectionID({0})", self.0)
    }
}

#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq, Serialize, Deserialize)]
pub enum IntersectionType {
    StopSign,
    TrafficSignal,
    Border,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct Intersection {
    pub id: IntersectionID,
    // This needs to be in clockwise orientation, or later rendering of sidewalk corners breaks.
    pub polygon: Polygon,
    pub turns: Vec<TurnID>,

    pub intersection_type: IntersectionType,
    pub label: Option<String>,
    pub stable_id: raw_data::StableIntersectionID,

    // Note that a lane may belong to both incoming_lanes and outgoing_lanes.
    // TODO narrow down when and why. is it just sidewalks in weird cases?
    pub incoming_lanes: Vec<LaneID>,
    pub outgoing_lanes: Vec<LaneID>,

    pub roads: BTreeSet<RoadID>,
}

impl Intersection {
    pub fn is_dead_end(&self) -> bool {
        self.roads.len() == 1
    }

    pub fn is_degenerate(&self) -> bool {
        self.roads.len() == 2
    }

    pub fn get_incoming_lanes(&self, map: &Map, lt: LaneType) -> Vec<LaneID> {
        self.incoming_lanes
            .iter()
            .filter(|l| map.get_l(**l).lane_type == lt)
            .cloned()
            .collect()
    }

    pub fn get_outgoing_lanes(&self, map: &Map, lt: LaneType) -> Vec<LaneID> {
        self.outgoing_lanes
            .iter()
            .filter(|l| map.get_l(**l).lane_type == lt)
            .cloned()
            .collect()
    }

    pub fn get_zorder(&self, map: &Map) -> isize {
        // TODO Not sure min makes sense -- what about a 1 and a 0? Prefer the nonzeros. If there's
        // a -1 and a 1... need to see it to know what to do.
        self.roads
            .iter()
            .map(|r| map.get_r(*r).get_zorder())
            .min()
            .unwrap()
    }

    pub fn get_rank(&self, map: &Map) -> usize {
        self.roads
            .iter()
            .map(|r| map.get_r(*r).get_rank())
            .max()
            .unwrap()
    }

    pub fn get_roads_sorted_by_incoming_angle(&self, all_roads: &Vec<Road>) -> Vec<RoadID> {
        let center = self.polygon.center();
        let mut roads: Vec<RoadID> = self.roads.iter().cloned().collect();
        roads.sort_by_key(|id| {
            let r = &all_roads[id.0];
            let endpt = if r.src_i == self.id {
                r.center_pts.first_pt()
            } else if r.dst_i == self.id {
                r.center_pts.last_pt()
            } else {
                unreachable!();
            };
            endpt.angle_to(center).normalized_degrees() as i64
        });
        roads
    }

    pub fn dump_debug(&self) {
        println!("{}", abstutil::to_json(self));
    }
}
