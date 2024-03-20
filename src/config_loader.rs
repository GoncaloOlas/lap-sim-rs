use std::fs::File;
use std::io::Read;
use toml;
use serde_derive::Deserialize;

// Define a struct that matches the structure of your TOML file
#[derive(Deserialize, Debug)]
struct Vehicle {
    revision: Revision,
    aero: Aero,
    battery: Battery,
    cooling: Cooling,
    driveline: Driveline,
    driver: Driver,
    hv: HV,
    lv: LV,
    motor: Motor,
    tctv: TCTV,
    tires: Tires,
    vd: VD,
    suspension: Suspension,
}

#[derive(Debug, Deserialize)]
struct Revision {
    value: u32,
}

#[derive(Debug, Deserialize)]
struct Aero {
    cl_profile: f64,
    cd_profile: f64,
    roh: f64,
    area: f64,
}

#[derive(Debug, Deserialize)]
struct Battery {
    soc_current: f64,
    cell_s: f64,
    cell_p: f64,
    cell_cap: f64,
    pack_volts: f64,
}

#[derive(Debug, Deserialize)]
struct Cooling{
    air_temp: f64,
    system_p_loss: f64,
    coolant_temp: f64,
    coolant_temp_max: f64,
    fan_max_draw: f64,
    fan_max_rate: f64,
    fan_count: f64,
    pump_max_draw: f64,
    pump_max_rate: f64,
}

#[derive(Debug, Deserialize)]
struct Driveline{
    motor_type: f64,
    configuration: f64,
    mechanical_efficiency: f64,
    gear_ratio: f64,
    moment_inertia: f64,
    dif_locking_coeff: f64,
    chain_tension: f64,
    viscous_coef: f64,
    max_motor_torque: f64,
}

#[derive(Debug, Deserialize)]
struct Driver{
    regen_en: f64,
    tc_en: f64,
    lc_en: f64,
    brake_bal: f64,
}

#[derive(Debug, Deserialize)]
struct HV{
    cable_r: f64,
    cable_z: f64,
    dc_link_cap: f64,
    dcdc_input_cap: f64,
}

#[derive(Debug, Deserialize)]
struct LV{
    lv_voltage: f64,
    dcdc_eff: f64,
    pump_eff: f64,
    batt_fan_eff: f64,
    fan_max_draw: f64,
    pump_max_draw: f64,
    lv_draw: f64,
}

#[derive(Debug, Deserialize)]
struct Motor{
    lambda_m: f64,
    r_s: f64,
    max_motor_power: f64,
    max_motor_current: f64,
    max_torque: f64,
    power_limit: f64,
    kv: f64,
    dc_link_cap: f64,
}

#[derive(Debug, Deserialize)]
struct TCTV{
    slip_ratio: f64,
    tire_angle: f64,
}

#[derive(Debug, Deserialize)]
struct Tires{    
    fz: f64,
    fz_nom: f64,
    fx: f64,
    fy: f64,
    pc_x_mat: f64,
    pc_y_mat: f64,
    pc_x: f64,
    pc_y: f64,
}

#[derive(Debug, Deserialize)]
struct VD{
    vehicle_mass: f64,
    wheel_radius: f64,
    cg_x: f64,
    cg_y: f64,
    cg_z: f64,
    wheelbase: f64,
    track_front: f64,
    track_rear: f64,
    tire_coef: f64,
    num_motor: f64,
}

#[derive(Debug, Deserialize)]
struct Suspension{
    ks: f64,
    bs: f64,
    motion_ratio: f64,
    m_unsp: f64,
    m_sp: f64,
}

pub fn load_vehicle_config() -> Vehicle{
    let mut file = File::open("src/configs/vehicle.toml").expect("Could not open vehicle.toml file");
    let mut contents = String::new();
    file.read_to_string(&mut contents).expect("Could not read vehicle.toml file");

    let vehicle: Vehicle = toml::from_str(&contents).expect("Could not deserialize vehicle.toml");

    vehicle
}

#[derive(Deserialize, Debug)]
pub struct Simulation{
    start_sim_time: f64,
    timestep: f64,
}

pub fn load_simulation_config() -> Simulation{
    let mut file = File::open("src/configs/simulation.toml").expect("Could not open simulation.toml file");
    let mut contents = String::new();
    file.read_to_string(&mut contents).expect("Could not read simulation.toml file");

    let simulation_config: Simulation = toml::from_str(&contents).expect("Could not deserialize simulation.toml");
    
    simulation_config
}