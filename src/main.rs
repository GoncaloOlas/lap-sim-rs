mod config_loader;
mod track;
mod vehicle_dynamics;
use crate::vehicle_dynamics::vehicle_dyn;

fn main() {
    let simulation_config = config_loader::load_simulation_config();
    let settings = config_loader::load_vehicle_config();
    let track = track::Track::new("src/tracks/autocross.csv").unwrap();
    // Create a new instance of vehicle_dyn
    let vehicle = vehicle_dyn::new(settings); // replace with the correct function and arguments to create a new vehicle_dyn

    let max_table = vehicle_dyn::max_corner_speed(&vehicle, track.raw_track);




}