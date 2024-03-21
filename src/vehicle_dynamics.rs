// use crate::config_loader::*;

// impl Vehicle {
//     pub fn weight_transfer_variable(&self, rho: f64) -> (f64, f64, f64, f64) {
//         let cgz = self.vd.cg_xyz[2];
        
//         let v_max = 10.0; // Define v_max with a specific value
//         let lat_accel_capacity = v_max.powi(2) / rho;
        
//         let longitudinal = 0.0; // self.accel[0]*mass*cgz/wheelbase; positive rear
//         let front_lateral = lat_accel_capacity*self.vehicle_mass*cgz/self.track_front; // positive right
//         let rear_lateral = lat_accel_capacity*self.vehicle_mass*cgz/self.track_rear;
        
//         let fr = -longitudinal + front_lateral + (0.25 * self.vehicle_mass);
//         let fl = -longitudinal - front_lateral + (0.25 * self.vehicle_mass);
//         let rr = longitudinal + rear_lateral + (0.25 * self.vehicle_mass);
//         let rl = longitudinal - rear_lateral + (0.25 * self.vehicle_mass);
        
//         (fr, fl, rr, rl)
//     }

//     pub fn max_corner_speed(&self, track_table: Vec<f64>) -> Vec<f64> {
//         let mut max_speed_table = vec![0.0; track_table.len()]; // Pre-allocate size of table

//         for i in 0..track_table.len() {
//             if track_table[i] != 0.0 { // True if line in track table is a corner not a straight
//                 let rho = track_table[i]; // Corner radius for current corner
//                 let (fr, fl, rr, rl) = self.weight_transfer_variable(rho); // Determine weight transfer in terms of v_max
//                 max_speed_table[i] = (rho * (self.tire_coef * (fr + fl + rr + rl)) / self.vehicle_mass).sqrt();
//             } else { // If line of track table is a straight
//                 max_speed_table[i] = 0.0; // Placeholder for v_max on straightaway
//             }
//         }

//         max_speed_table
//     }
// }

use nalgebra as na;

pub struct vehicle_dyn{
    vehicle_mass: f64,
    wheel_radius: f64,
    cg_xyz: na::Vector3<f64>,
    wheelbase: f64,
    track_front: f64,
    track_rear: f64,
    acceleration: na::Vector3<f64>,
    tire_coef: f64,
    num_motor: usize,
    force_matrix: Vec<na::Vector3<f64>>,
    force_location_matrix: Vec<na::Vector3<f64>>,
    pos: na::Vector3<f64>,
    ang_pos: f64,
    vel: na::Vector3<f64>,
    ang_vel: na::Vector3<f64>,
    accel: na::Vector3<f64>,
    ang_accel: na::Vector3<f64>,
    velocity: f64,
    max_velocity: f64,
    wheel_torque: f64,
    time_p: f64,
    pos_p: Vec<na::Vector3<f64>>,
    ang_pos_p: f64,
    vel_p: Vec<na::Vector3<f64>>,
    ang_vel_p: Vec<na::Vector3<f64>>,
    accel_p: Vec<na::Vector3<f64>>,
    ang_accel_p: Vec<na::Vector3<f64>>,
    k: usize,
    last_update_time: f64,
}

impl vehicle_dyn {
    pub fn new(raw_vals: [f64; 10]) -> Self {
        Self {
            k: 1,
            vehicle_mass: raw_vals[0],
            wheel_radius: raw_vals[1],
            cg_xyz: na::Vector3::from_column_slice(&raw_vals[2..5]),
            wheelbase: raw_vals[5],
            track_front: raw_vals[6],
            track_rear: raw_vals[7],
            tire_coef: raw_vals[8],
            num_motor: raw_vals[9] as usize,
            acceleration: na::Vector3::zeros(),
            force_matrix: Vec::new(),
            force_location_matrix: Vec::new(),
            pos: na::Vector3::zeros(),
            ang_pos: 0.0,
            vel: na::Vector3::zeros(),
            ang_vel: na::Vector3::zeros(),
            accel: na::Vector3::zeros(),
            ang_accel: na::Vector3::zeros(),
            velocity: 0.0,
            max_velocity: 0.0,
            wheel_torque: 0.0,
            time_p: 0.0,
            pos_p: vec![na::Vector3::zeros(); 3],
            ang_pos_p: 0.0,
            vel_p: vec![na::Vector3::zeros(); 3],
            ang_vel_p: vec![na::Vector3::zeros(); 3],
            accel_p: vec![na::Vector3::zeros(); 3],
            ang_accel_p: vec![na::Vector3::zeros(); 3],
            last_update_time: 0.0,
        }
    }

    pub fn global_to_local(&self, global_vec: na::Vector3<f64>) -> na::Vector3<f64> {
        let rotation_matrix = na::Matrix2::new(self.ang_pos.cos(), self.ang_pos.sin(),
                                               -self.ang_pos.sin(), self.ang_pos.cos());
        rotation_matrix * global_vec
    }

    pub fn local_to_global(&self, local_vec: na::Vector3<f64>) -> na::Vector3<f64> {
        let rotation_matrix = na::Matrix2::new(self.ang_pos.cos(), -self.ang_pos.sin(),
                                               self.ang_pos.sin(), self.ang_pos.cos());
        rotation_matrix * local_vec
    }

    pub fn weight_transfer(&self) -> (f64, f64, f64, f64) {
        let cgz = self.cg_xyz[2];
        let longitudinal = self.accel[0] * self.vehicle_mass * cgz / self.wheelbase;
        let front_lateral = self.accel[1] * self.vehicle_mass * cgz / self.track_front;
        let rear_lateral = self.accel[1] * self.vehicle_mass * cgz / self.track_rear;

        let dfr = -longitudinal + front_lateral;
        let dfl = -longitudinal - front_lateral;
        let drr = longitudinal + rear_lateral;
        let drl = longitudinal - rear_lateral;

        (dfr, dfl, drr, drl)
    }

    pub fn update_position(&mut self, sim_time: f64) {
        let time_step = sim_time - self.last_update_time;
        self.last_update_time = sim_time;

        let a2 = self.accel;
        let a1 = self.accel_p[0];
        let a0 = self.accel_p[1];
        let v1 = self.vel;
        self.vel = runge_kutta(v1, a0, a1, a2, time_step);

        let aa2 = self.ang_accel;
        let aa1 = self.ang_accel_p[0];
        let aa0 = self.ang_accel_p[1];
        let av1 = self.ang_vel;
        self.ang_vel = runge_kutta(av1, aa0, aa1, aa2, time_step);
        
        // Continue with the rest of the implementation
    }

    pub fn weight_transfer_variable(&self, rho: f64) -> (f64, f64, f64, f64) {
        let cgz = self.cg_xyz[2];
        let lat_accel_capacity = self.max_velocity.powi(2) / rho;

        let longitudinal = 0.0;
        let front_lateral = lat_accel_capacity * self.vehicle_mass * cgz / self.track_front;
        let rear_lateral = lat_accel_capacity * self.vehicle_mass * cgz / self.track_rear;

        let fr = -longitudinal + front_lateral + (0.25 * self.vehicle_mass);
        let fl = -longitudinal - front_lateral + (0.25 * self.vehicle_mass);
        let rr = longitudinal + rear_lateral + (0.25 * self.vehicle_mass);
        let rl = longitudinal - rear_lateral + (0.25 * self.vehicle_mass);

        (fr, fl, rr, rl)
    }

    pub fn max_corner_speed(&self, track_table: Vec<Vec<f64>>) -> Vec<f64> {
        let mut max_speed_table = vec![0.0; track_table.len()];

        for i in 0..track_table.len() {
            if track_table[i][2] != 0.0 {
                let rho = track_table[i][2];
                let (fr, fl, rr, rl) = self.weight_transfer_variable(rho);
                max_speed_table[i] = ((rho * (self.tire_coef * (fr + fl + rr + rl)) / self.vehicle_mass).sqrt());
            }
        }

        max_speed_table
    }

    pub fn vd_main(&mut self, ae: &Ae, dr: &Dr, mo: &Mo, track_table: &Vec<Vec<f64>>, max_corner_table: &Vec<f64>, dt: f64) {
            let mut element_remain__: f64 = 0.0;
            let mut in_element__: bool = false;
            let mut last_element__: bool = false;
    
            if element_remain__.abs() < std::f64::EPSILON {
                element_remain__ = track_table[self.k][1];
                in_element__ = true;
                last_element__ = false;
            }
    
            if !in_element__ {
                self.k += 1;
                element_remain__ = track_table[self.k][1];
                in_element__ = true;
                if self.k >= track_table.len() {
                    last_element__ = true;
                }
            }
    
            let mut rr_force = (0.005 + (1.0 / 1.379) * (0.01 + 0.0095 * ((self.velocity / 100.0).powi(2)))) * self.vehicle_mass * 9.81;
    
            if track_table[self.k][2] != 0.0 {
                self.velocity = max_corner_table[self.k];
                element_remain__ -= dt * self.velocity;
                if element_remain__.abs() < std::f64::EPSILON {
                    in_element__ = false;
                }
                self.wheel_torque = (rr_force + ae.drag_force) / self.num_motor as f64 * self.wheel_radius;
            } else {
                let corner_entry_v = (self.velocity.powi(2) - 19.62 * element_remain__).sqrt();
                if corner_entry_v.is_finite() && self.velocity >= corner_entry_v && !last_element__ {
                    let new_velocity = self.velocity - (9.81 * self.tire_coef * dt);
                    let deccel = 9.81 * self.tire_coef;
                    let fric_force = ((deccel / self.tire_coef) * self.vehicle_mass) / 4.0;
                    self.wheel_torque = -fric_force / self.wheel_radius;
                } else {
                    let motor_force = dr.max_motor_torque / self.wheel_radius;
                    let fric_force = self.tire_coef * self.vehicle_mass / 4.0;
                    if motor_force <= fric_force {
                        let accel_force = motor_force * self.num_motor as f64;
                        self.wheel_torque = dr.max_motor_torque;
                    } else {
                        let accel_force = fric_force * self.num_motor as f64;
                        self.wheel_torque = fric_force / self.wheel_radius;
                    }
                    self.velocity += ((((accel_force * self.num_motor as f64) - rr_force - ae.drag_force) / self.vehicle_mass) * dt);
                }
                element_remain__ -= dt * self.velocity;
                if element_remain__.abs() < std::f64::EPSILON {
                    in_element__ = false;
                }
            }
    
            if self.velocity > self.max_velocity {
                self.max_velocity = self.velocity;
            }
        }
    }
    
    struct Ae {
        drag_force: f64,
    }
    
    struct Dr {
        max_motor_torque: f64,
    }
    
    struct Mo {}
    
    fn runge_kutta(v1: na::Vector3<f64>, a0: na::Vector3<f64>, a1: na::Vector3<f64>, a2: na::Vector3<f64>, time_step: f64) -> na::Vector3<f64> {
        // Placeholder for Runge-Kutta integration
        // Implement your own Runge-Kutta integration method here
        // This is just a dummy implementation
        v1 + a0 * time_step + a1 * time_step + a2 * time_step
    }
    