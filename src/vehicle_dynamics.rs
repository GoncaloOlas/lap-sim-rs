use crate::config_loader::Vehicle::VD;

impl Vehicle {
    pub fn weight_transfer_variable(&self, rho: f64) -> (f64, f64, f64, f64) {
        let cgz = self.cg_xyz[2];
        
        let v_max = 10.0; // Define v_max with a specific value
        let lat_accel_capacity = v_max.powi(2) / rho;
        
        let longitudinal = 0.0; // self.accel[0]*mass*cgz/wheelbase; positive rear
        let front_lateral = lat_accel_capacity*self.vehicle_mass*cgz/self.track_front; // positive right
        let rear_lateral = lat_accel_capacity*self.vehicle_mass*cgz/self.track_rear;
        
        let fr = -longitudinal + front_lateral + (0.25 * self.vehicle_mass);
        let fl = -longitudinal - front_lateral + (0.25 * self.vehicle_mass);
        let rr = longitudinal + rear_lateral + (0.25 * self.vehicle_mass);
        let rl = longitudinal - rear_lateral + (0.25 * self.vehicle_mass);
        
        (fr, fl, rr, rl)
    }

    pub fn max_corner_speed(&self, track_table: Vec<f64>) -> Vec<f64> {
        let mut max_speed_table = vec![0.0; track_table.len()]; // Pre-allocate size of table

        for i in 0..track_table.len() {
            if track_table[i] != 0.0 { // True if line in track table is a corner not a straight
                let rho = track_table[i]; // Corner radius for current corner
                let (fr, fl, rr, rl) = self.weight_transfer_variable(rho); // Determine weight transfer in terms of v_max
                max_speed_table[i] = (rho * (self.tire_coef * (fr + fl + rr + rl)) / self.vehicle_mass).sqrt();
            } else { // If line of track table is a straight
                max_speed_table[i] = 0.0; // Placeholder for v_max on straightaway
            }
        }

        max_speed_table
    }
}