mod config_loader;

fn main() {
    let settings = config_loader::load_vehicle_config();
    println!("{:?}", settings);
}