use crate::runner::TestRunner;
use abstutil;
use convert_osm;
use map_model;

pub fn run(t: &mut TestRunner) {
    t.run_slow("convert_osm_twice", |_| {
        let flags = convert_osm::Flags {
            osm: "../data/input/montlake.osm".to_string(),
            traffic_signals: "../data/input/traffic_signals.kml".to_string(),
            residential_buildings: "../data/input/residential_buildings.kml".to_string(),
            parking_shapes: "../data/shapes/blockface.bin".to_string(),
            gtfs: "../data/input/google_transit_2018_18_08".to_string(),
            neighborhoods: "../data/input/neighborhoods.geojson".to_string(),
            clip: "../data/polygons/montlake.poly".to_string(),
            output: "convert_osm_twice.bin".to_string(),
            fast_dev: false,
        };

        let map1 = convert_osm::convert(&flags, &mut abstutil::Timer::throwaway());
        let map2 = convert_osm::convert(&flags, &mut abstutil::Timer::throwaway());

        if map1 != map2 {
            // TODO tmp files
            abstutil::write_json("map1.json", &map1).unwrap();
            abstutil::write_json("map2.json", &map2).unwrap();
            panic!("map1.json and map2.json differ");
        }
    });

    t.run_slow("raw_to_map_twice", |_| {
        let map1 = map_model::Map::new(
            "../data/raw_maps/montlake.bin",
            &mut abstutil::Timer::throwaway(),
        )
        .unwrap();
        let map2 = map_model::Map::new(
            "../data/raw_maps/montlake.bin",
            &mut abstutil::Timer::throwaway(),
        )
        .unwrap();

        if abstutil::to_json(&map1) != abstutil::to_json(&map2) {
            // TODO tmp files
            abstutil::write_json("map1.json", &map1).unwrap();
            abstutil::write_json("map2.json", &map2).unwrap();
            panic!("map1.json and map2.json differ");
        }
    });

    t.run_slow("bigger_map_loads", |_| {
        map_model::Map::new(
            "../data/raw_maps/23rd.bin",
            &mut abstutil::Timer::throwaway(),
        )
        .expect("23rd broke");
    });

    t.run_slow("biggest_map_loads", |_| {
        map_model::Map::new(
            "../data/raw_maps/huge_seattle.bin",
            &mut abstutil::Timer::throwaway(),
        )
        .expect("huge_seattle broke");
    });
}
