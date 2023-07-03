extern crate nalgebra as na;

use na::{Matrix2, UnitVector2, Vector2};

#[allow(non_snake_case)]
pub struct ILOS {
    yaw_angle: f64,
    yaw_rate: f64,
    integral_state: f64,
    S: Matrix2<f64>,
    k: f64,
    c: f64,
}

impl Default for ILOS {
    fn default() -> Self {
        Self::new(1.0, 1.0)
    }
}

impl ILOS {
    pub fn new(prop_gain: f64, integral_gain: f64) -> ILOS {
        ILOS {
            yaw_angle: 0.0,
            yaw_rate: 0.0,
            integral_state: 0.0,
            S: Matrix2::new(0.0, -1.0, 1.0, 0.0),
            k: prop_gain,
            c: integral_gain,
        }
    }

    pub fn update(
        &mut self,
        pos: &Vector2<f64>,
        pos_d: &Vector2<f64>,
        tau: &UnitVector2<f64>,
        dt: f64,
    ) {
        let k = self.k;
        let c = self.c;

        let cross_track_err = (pos - pos_d).dot(&(self.S * tau.into_inner()));
        let mu = (tau.into_inner()
            - (k * cross_track_err + c * self.integral_state) * self.S * tau.into_inner())
            / (1.0 + (k * cross_track_err + c * self.integral_state).powi(2)).sqrt();

        let yaw_angle_prev = self.yaw_angle;
        self.yaw_angle = mu[1].atan2(mu[0]);
        self.yaw_rate = ssa(ssa(self.yaw_angle) - ssa(yaw_angle_prev)) / dt;

        let alpha_dot = k * cross_track_err
            / (1.0 + (k * cross_track_err + c * self.integral_state).powi(2)).sqrt();
        self.integral_state += alpha_dot * dt;
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
