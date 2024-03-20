mod config_loader;
mod track;
mod vehicle_dynamics;

fn main() {
    let simulation_config = config_loader::load_simulation_config();
    let settings = config_loader::load_vehicle_config();
    let track = track::Track::new("src/tracks/autocross.csv").unwrap();





}