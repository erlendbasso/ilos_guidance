extern crate nalgebra as na;

use std::f64::consts::SQRT_2;

use na::{UnitVector2, Vector2};

use crate::paths::path::Path;

#[derive(Clone, Copy)]
pub struct BgdParameters {
    s_bar: f64,
    sigma: f64,
    mu: f64,
}

impl BgdParameters {
    pub fn new(s_bar: f64, sigma: f64, mu: f64) -> BgdParameters {
        BgdParameters { s_bar, sigma, mu }
    }
}

#[derive(Clone, Copy)]
pub struct Lemniscate {
    height: f64,         // half of the height of the lemniscate
    width: f64,          // half of the width of the lemniscate
    pos_0: Vector2<f64>, // center of the lemniscate

    // Backtracking gradient descent parameters
    bgd_parameters: BgdParameters,
}

impl Lemniscate {
    pub fn new(
        height: f64,
        width: f64,
        pos_0: Vector2<f64>,
        bgd_parameters: BgdParameters,
    ) -> Lemniscate {
        Lemniscate {
            height,
            width,
            pos_0,
            bgd_parameters,
        }
    }

    fn comp_derivative(&self, theta: f64) -> Vector2<f64> {
        let x = 2.0
            * self.height
            * ((2.0 * SQRT_2 * f64::cos(2.0 * theta)) / (1.0 + f64::powi(f64::sin(theta), 2))
                - 2.0
                    * ((SQRT_2 * f64::sin(2.0 * theta))
                        / f64::powi(1.0 + f64::powi(f64::sin(theta), 2), 2))
                    * f64::sin(theta)
                    * f64::cos(theta));
        let y = 2.0
            * self.width
            * ((-f64::sin(theta)) / (1.0 + f64::powi(f64::sin(theta), 2))
                - 2.0
                    * f64::sin(theta)
                    * (f64::cos(theta) / f64::powi(1.0 + f64::powi(f64::sin(theta), 2), 2))
                    * f64::cos(theta));

        Vector2::new(x, y)
    }

    fn backtracking_gradient_descent(&self, pos: Vector2<f64>, theta_0: f64) -> f64 {
        let along_track_error = |pos: &Vector2<f64>, theta: f64| -> f64 {
            (pos - self.comp_pos(theta)).dot(&self.comp_derivative(theta).normalize())
        };

        let theta_plus = |theta: f64, s: f64| -> f64 {
            theta + s * along_track_error(&pos, theta) / self.comp_derivative(theta).norm()
        };

        let mut theta = theta_0;

        let s_bar = self.bgd_parameters.s_bar;
        let sigma = self.bgd_parameters.sigma;
        let mu = self.bgd_parameters.mu;

        while along_track_error(&pos, theta).abs() > 10e-3 {
            let mut s = s_bar;
            while (pos - self.comp_pos(theta_plus(theta, s))).norm_squared()
                - (pos - self.comp_pos(theta)).norm_squared()
                > -2.0 * s * sigma * f64::powi(along_track_error(&pos, theta), 2)
            {
                s *= mu;
            }
            theta = theta_plus(theta, s);
        }
        theta
    }

    pub fn comp_theta_bgd(&self, pos: &Vector2<f64>, theta_0: f64) -> f64 {
        self.backtracking_gradient_descent(*pos, theta_0)
    }
}

impl Path for Lemniscate {
    fn comp_theta(&mut self, _pos: &Vector2<f64>) -> f64 {
        unimplemented!()
    }

    fn comp_pos(&self, theta: f64) -> Vector2<f64> {
        let denominator = 1.0 + f64::powi(f64::sin(theta), 2);
        let x = 2.0 * self.height * f64::sqrt(2.0) * f64::sin(2.0 * theta) / denominator;
        let y = 2.0 * self.width * f64::cos(theta) / denominator;

        Vector2::new(x, y) + self.pos_0
    }

    fn comp_tangent(&self, theta: f64) -> UnitVector2<f64> {
        let x = 2.0
            * self.height
            * ((2.8284271247461903 * f64::cos(2.0 * theta))
                / (1.0 + f64::powi(f64::sin(theta), 2))
                - 2.0
                    * ((SQRT_2 * f64::sin(2.0 * theta))
                        / f64::powi(1.0 + f64::powi(f64::sin(theta), 2), 2))
                    * f64::sin(theta)
                    * f64::cos(theta));
        let y = 2.0
            * self.width
            * ((-f64::sin(theta)) / (1.0 + f64::powi(f64::sin(theta), 2))
                - 2.0
                    * f64::sin(theta)
                    * (f64::cos(theta) / f64::powi(1.0 + f64::powi(f64::sin(theta), 2), 2))
                    * f64::cos(theta));

        UnitVector2::new_normalize(Vector2::new(x, y))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_lemniscate() {
        let lemniscate = Lemniscate::new(0.5, 0.5, Vector2::new(0.0, 0.0));
        let pos = lemniscate.comp_pos(13.37);
        let tang = lemniscate.comp_tangent(13.37);

        assert_eq!(pos, Vector2::new(0.9308709375969799, 0.4571770103950108));
    }
}
