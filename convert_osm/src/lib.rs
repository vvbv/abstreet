mod clip;
mod neighborhoods;
mod osm;
mod remove_disconnected;
mod split_ways;

use abstutil::Timer;
use geom::{Distance, FindClosest, LonLat, PolyLine, Pt2D};
use kml::ExtraShapes;
use map_model::{raw_data, IntersectionType, LANE_THICKNESS};
use std::fs::File;
use std::io::{BufRead, BufReader};
use structopt::StructOpt;

const MAX_DIST_BTWN_INTERSECTION_AND_SIGNAL: Distance = Distance::const_meters(50.0);
const MAX_DIST_BTWN_BLDG_PERMIT_AND_BLDG: Distance = Distance::const_meters(10.0);

#[derive(StructOpt, Debug)]
#[structopt(name = "convert_osm")]
pub struct Flags {
    /// OSM XML file to read
    #[structopt(long = "osm")]
    pub osm: String,

    /// KML with traffic signals. Optional.
    #[structopt(long = "traffic_signals", default_value = "")]
    pub traffic_signals: String,

    /// KML with residential building permits. Optional.
    #[structopt(long = "residential_buildings", default_value = "")]
    pub residential_buildings: String,

    /// ExtraShapes file with blockface, produced using the kml crate. Optional.
    #[structopt(long = "parking_shapes", default_value = "")]
    pub parking_shapes: String,

    /// GTFS directory. Optional.
    #[structopt(long = "gtfs", default_value = "")]
    pub gtfs: String,

    /// Neighborhood GeoJSON path. Optional.
    #[structopt(long = "neighborhoods", default_value = "")]
    pub neighborhoods: String,

    /// Osmosis clipping polgon
    #[structopt(long = "clip")]
    pub clip: String,

    /// Output .bin path
    #[structopt(long = "output")]
    pub output: String,

    /// Disable blockface
    #[structopt(long = "fast_dev")]
    pub fast_dev: bool,
}

pub fn convert(flags: &Flags, timer: &mut abstutil::Timer) -> raw_data::Map {
    let mut map = split_ways::split_up_roads(osm::osm_to_raw_roads(&flags.osm, timer), timer);
    map.boundary_polygon = read_osmosis_polygon(&flags.clip);
    clip::clip_map(&mut map, timer);
    remove_disconnected::remove_disconnected_roads(&mut map, timer);

    if flags.fast_dev {
        return map;
    }
    // Do this after removing stuff.
    map.compute_gps_bounds();

    if !flags.residential_buildings.is_empty() {
        handle_residences(&mut map, &flags.residential_buildings, timer);
    }
    if !flags.parking_shapes.is_empty() {
        use_parking_hints(&mut map, &flags.parking_shapes, timer);
    }
    if !flags.traffic_signals.is_empty() {
        handle_traffic_signals(&mut map, &flags.traffic_signals, timer);
    }
    if !flags.gtfs.is_empty() {
        timer.start("load GTFS");
        map.bus_routes = gtfs::load(&flags.gtfs).unwrap();
        timer.stop("load GTFS");
    }

    if !flags.neighborhoods.is_empty() {
        timer.start("convert neighborhood polygons");
        let map_name = abstutil::basename(&flags.output);
        neighborhoods::convert(&flags.neighborhoods, map_name, &map.gps_bounds);
        timer.stop("convert neighborhood polygons");
    }

    map
}

fn use_parking_hints(map: &mut raw_data::Map, path: &str, timer: &mut Timer) {
    timer.start("apply parking hints");
    println!("Loading blockface shapes from {}", path);
    let shapes: ExtraShapes = abstutil::read_binary(path, timer).expect("loading blockface failed");

    // Match shapes with the nearest road + direction (true for forwards)
    let mut closest: FindClosest<(raw_data::StableRoadID, bool)> =
        FindClosest::new(&map.gps_bounds.to_bounds());
    for (id, r) in &map.roads {
        let pts = PolyLine::new(map.gps_bounds.must_convert(&r.points));
        closest.add(
            (*id, true),
            pts.shift_right(LANE_THICKNESS).get(timer).points(),
        );
        closest.add(
            (*id, false),
            pts.shift_left(LANE_THICKNESS).get(timer).points(),
        );
    }

    'SHAPE: for s in shapes.shapes.into_iter() {
        let mut pts: Vec<Pt2D> = Vec::new();
        for pt in s.points.into_iter() {
            if let Some(pt) = Pt2D::from_gps(pt, &map.gps_bounds) {
                pts.push(pt);
            } else {
                continue 'SHAPE;
            }
        }
        if pts.len() > 1 {
            // The blockface line endpoints will be close to other roads, so match based on the
            // middle of the blockface.
            // TODO Long blockfaces sometimes cover two roads. Should maybe find ALL matches within
            // the threshold distance?
            let middle = PolyLine::new(pts).middle();
            if let Some(((r, fwds), _)) = closest.closest_pt(middle, LANE_THICKNESS * 5.0) {
                let category = s.attributes.get("PARKING_CATEGORY");
                let has_parking = category != Some(&"None".to_string())
                    && category != Some(&"No Parking Allowed".to_string());
                // Blindly override prior values.
                if fwds {
                    map.roads.get_mut(&r).unwrap().parking_lane_fwd = has_parking;
                } else {
                    map.roads.get_mut(&r).unwrap().parking_lane_back = has_parking;
                }
            }
        }
    }
    timer.stop("apply parking hints");
}

fn handle_traffic_signals(map: &mut raw_data::Map, path: &str, timer: &mut Timer) {
    timer.start("handle traffic signals");
    for shape in kml::load(path, &map.gps_bounds, timer)
        .expect("loading traffic signals failed")
        .shapes
        .into_iter()
    {
        // See https://www.seattle.gov/Documents/Departments/SDOT/GIS/Traffic_Signals_OD.pdf
        if shape.points.len() > 1 {
            panic!("Traffic signal has multiple points: {:?}", shape);
        }
        let pt = shape.points[0];
        if map.gps_bounds.contains(pt) {
            // TODO use a quadtree or some better way to match signals to the closest
            // intersection
            let closest_intersection = map
                .intersections
                .values_mut()
                .min_by_key(|i| pt.gps_dist_meters(i.point))
                .unwrap();
            let dist = pt.gps_dist_meters(closest_intersection.point);
            if dist <= MAX_DIST_BTWN_INTERSECTION_AND_SIGNAL {
                if closest_intersection.intersection_type == IntersectionType::TrafficSignal {
                    println!("WARNING: {:?} already has a traffic signal, but there's another one that's {} from it", closest_intersection, dist);
                }
                closest_intersection.intersection_type = IntersectionType::TrafficSignal;
            }
        }
    }
    timer.stop("handle traffic signals");
}

fn handle_residences(map: &mut raw_data::Map, path: &str, timer: &mut Timer) {
    timer.start("match residential permits with buildings");

    let mut closest: FindClosest<usize> = FindClosest::new(&map.gps_bounds.to_bounds());
    for (idx, b) in map.buildings.iter().enumerate() {
        // TODO Ew, have to massage into Pt2D.
        closest.add_gps(idx, &b.points, &map.gps_bounds);
    }

    let shapes = kml::load(path, &map.gps_bounds, timer)
        .expect("loading residential buildings failed")
        .shapes;
    timer.start_iter("handle residential permits", shapes.len());
    for shape in shapes.into_iter() {
        timer.next();
        if shape.points.len() > 1 {
            panic!(
                "Residential building permit has multiple points: {:?}",
                shape
            );
        }
        let pt = shape.points[0];
        if !map.gps_bounds.contains(pt) {
            continue;
        }
        if let Some(num) = shape
            .attributes
            .get("net_units")
            .and_then(|n| usize::from_str_radix(n, 10).ok())
        {
            if let Some((idx, _)) = closest.closest_pt(
                Pt2D::from_gps(pt, &map.gps_bounds).unwrap(),
                MAX_DIST_BTWN_BLDG_PERMIT_AND_BLDG,
            ) {
                // Just blindly override with the latest point. The dataset says multiple permits
                // per building might exist.
                map.buildings[idx].num_residential_units = Some(num);
            }
        }
    }
    timer.stop("match residential permits with buildings");
}

fn read_osmosis_polygon(path: &str) -> Vec<LonLat> {
    let mut pts: Vec<LonLat> = Vec::new();
    for (idx, maybe_line) in BufReader::new(File::open(path).unwrap())
        .lines()
        .enumerate()
    {
        if idx == 0 || idx == 1 {
            continue;
        }
        let line = maybe_line.unwrap();
        if line == "END" {
            break;
        }
        let parts: Vec<&str> = line.trim_start().split("    ").collect();
        assert!(parts.len() == 2);
        let lon = parts[0].parse::<f64>().unwrap();
        let lat = parts[1].parse::<f64>().unwrap();
        pts.push(LonLat::new(lon, lat));
    }
    pts
}
