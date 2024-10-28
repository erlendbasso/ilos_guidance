extern crate nalgebra as na;

use std::f64::consts::PI;

use na::{Matrix2, UnitVector2, Vector2};
use nalgebra::{stack, Rotation2};

#[allow(non_snake_case)]
pub struct ALOS {
    yaw_angle: f64,
    yaw_rate: f64,
    integral_state: f64,
    S: Matrix2<f64>,
    lookahead_distance: f64,
    ki: f64,
    saturation_limit: f64,
}

impl Default for ALOS {
    fn default() -> Self {
        Self::new(1.0, 1.0, 10.0)
    }
}

impl ALOS {
    pub fn new(lookahead_distance: f64, integral_gain: f64, saturation_limit: f64) -> ALOS {
        ALOS {
            yaw_angle: 0.0,
            yaw_rate: 0.0,
            integral_state: 0.0,
            S: Matrix2::new(0.0, -1.0, 1.0, 0.0),
            lookahead_distance,
            ki: integral_gain,
            saturation_limit,
        }
    }

    pub fn update(
        &mut self,
        pos: &Vector2<f64>,
        pos_d: &Vector2<f64>,
        tau: &UnitVector2<f64>,
        dt: f64,
    ) {
        let k = self.ki;

        let cross_track_err = (pos - pos_d).dot(&(self.S * tau.into_inner()));

        let rot_mat_beta = Rotation2::new(self.integral_state);
        let los_vector = Vector2::new(self.lookahead_distance, -cross_track_err).normalize();
        let mut alos_vector = rot_mat_beta.inverse() * los_vector;
        let alos_angle = f64::atan2(alos_vector[1], alos_vector[0]);

        let beta_dot = k * cross_track_err * self.lookahead_distance
            / f64::sqrt(self.lookahead_distance.powi(2) + cross_track_err.powi(2));

        self.integral_state += beta_dot * dt;
        // Check if the integral state is within the saturation limits
        self.integral_state = self
            .integral_state
            .clamp(-self.saturation_limit, self.saturation_limit);

        if alos_angle >= PI / 2.0 || alos_angle <= -PI / 2.0 {
            alos_vector = alos_vector[1].signum() * Vector2::new(0.0, 1.0);
            // We cannot let the integral state grow when we saturate the alos_vector
            self.integral_state =
                -alos_vector[1].signum() * f64::atan2(los_vector[0], los_vector[1]);
        }

        let rot_tau: Matrix2<f64> = stack![tau.into_inner(), self.S * tau.into_inner()];
        let control_law = rot_tau * alos_vector;

        // let yaw_angle_prev = self.yaw_angle;
        self.yaw_angle = f64::atan2(control_law[1], control_law[0]);
        // self.yaw_rate = ssa(ssa(self.yaw_angle) - ssa(yaw_angle_prev)) / dt;
        self.yaw_rate = 0.0;
    }

    pub fn set_gains(&mut self, lookahead_distance: f64, integral_gain: f64) {
        self.lookahead_distance = lookahead_distance;
        self.ki = integral_gain;
    }

    pub fn get_gains(&self) -> (f64, f64) {
        (self.lookahead_distance, self.ki)
    }

    pub fn get_references(&self) -> (f64, f64) {
        (self.yaw_angle, self.yaw_rate)
    }
}

pub fn ssa(ang: f64) -> f64 {
    let pi = core::f64::consts::PI;
    modulo(ang + pi, 2.0 * pi) - pi
}

fn modulo(m: f64, n: f64) -> f64 {
    (m % n + n) % n
}
