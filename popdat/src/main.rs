use structopt::StructOpt;

#[derive(StructOpt)]
#[structopt(name = "popdat")]
struct Flags {
    /// Stop after this many trips, for faster development
    #[structopt(long = "cap")]
    pub cap: Option<usize>,
}

fn main() {
    let flags = Flags::from_args();

    let mut timer = abstutil::Timer::new("creating popdat");
    let mut popdat = popdat::PopDat::import_all(&mut timer);

    // TODO Productionize this.
    // https://file.ac/cLdO7Hp_OB0/ has trips_2014.csv. https://file.ac/Xdjmi8lb2dA/ has the 2014
    // inputs.
    let (trips, parcels) = popdat::psrc::import_trips(
        "/home/dabreegster/Downloads/psrc/2014/landuse/parcels_urbansim.txt",
        "/home/dabreegster/Downloads/psrc/trips_2014.csv",
        &mut timer,
    )
    .unwrap();
    popdat.trips = trips;
    popdat.parcels = parcels;
    if let Some(n) = flags.cap {
        popdat.trips = popdat.trips.into_iter().take(n).collect();
    }

    abstutil::write_binary("../data/shapes/popdat.bin", &popdat).unwrap();
}
