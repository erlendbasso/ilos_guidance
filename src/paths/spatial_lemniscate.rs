extern crate nalgebra as na;

use std::f64::consts::SQRT_2;

use na::{UnitVector3, Vector3};

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
pub struct SpatialLemniscate {
    height: f64,         // half of the height of the lemniscate (x-direction)
    width: f64,          // half of the width of the lemniscate (y-direction)
    amplitude: f64,      // amplitude of the sinusoidal z-component
    pos_0: Vector3<f64>, // center of the lemniscate

    // Backtracking gradient descent parameters
    bgd_parameters: BgdParameters,
}

impl SpatialLemniscate {
    pub fn new(
        height: f64,
        width: f64,
        amplitude: f64,
        pos_0: Vector3<f64>,
        bgd_parameters: BgdParameters,
    ) -> SpatialLemniscate {
        SpatialLemniscate {
            height,
            width,
            amplitude,
            pos_0,
            bgd_parameters,
        }
    }

    fn comp_derivative(&self, theta: f64) -> Vector3<f64> {
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
        let z = self.amplitude * f64::cos(theta);

        Vector3::new(x, y, z)
    }

    fn backtracking_gradient_descent(&self, pos: Vector3<f64>, theta_0: f64) -> f64 {
        let along_track_error = |pos: &Vector3<f64>, theta: f64| -> f64 {
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

    pub fn comp_theta_bgd(&self, pos: &Vector3<f64>, theta_0: f64) -> f64 {
        self.backtracking_gradient_descent(*pos, theta_0)
    }

    pub fn comp_pos(&self, theta: f64) -> Vector3<f64> {
        let denominator = 1.0 + f64::powi(f64::sin(theta), 2);
        let x = 2.0 * self.height * f64::sqrt(2.0) * f64::sin(2.0 * theta) / denominator;
        let y = 2.0 * self.width * f64::cos(theta) / denominator;
        // z-offset is encoded in pos_0.z; only the oscillatory term remains here.
        let z = self.amplitude * f64::sin(theta);

        Vector3::new(x, y, z) + self.pos_0
    }

    pub fn comp_tangent(&self, theta: f64) -> UnitVector3<f64> {
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
        let z = self.amplitude * f64::cos(theta);

        UnitVector3::new_normalize(Vector3::new(x, y, z))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_spatial_lemniscate_z_component() {
        let lemniscate = SpatialLemniscate::new(
            0.5,
            0.5,
            1.0,
            Vector3::new(0.0, 0.0, 2.0),
            BgdParameters::new(1.0, 0.1, 0.5),
        );

        // At theta = 0, sin(0) = 0, so z should be pos_0.z + 0 = 2.0
        let pos_0 = lemniscate.comp_pos(0.0);
        assert!((pos_0.z - 2.0).abs() < 1e-10);

        // At theta = π/2, sin(π/2) = 1, so z should be pos_0.z + amplitude = 3.0
        let pos_pi2 = lemniscate.comp_pos(std::f64::consts::FRAC_PI_2);
        assert!((pos_pi2.z - 3.0).abs() < 1e-10);
    }
}
