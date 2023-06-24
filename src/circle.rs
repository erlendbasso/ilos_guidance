extern crate nalgebra as na;

use na::{Vector2, UnitVector2};

use crate::path::Path;

pub struct Circle {
    radius: f64,
    center: Vector2<f64>,
    q: f64,
}

impl Circle {
    pub fn new(radius: f64, center: Vector2<f64>, clockwise: bool) -> Circle {
        let q = if clockwise { -1.0 } else { 1.0 };
        Circle {
            radius,
            center,
            q,
        }
    }
}

impl Path for Circle {
    fn comp_theta(&self, pos: &Vector2<f64>) -> f64 {
        self.q * (pos[1] - self.center[1]).atan2(pos[0] - self.center[0])
        // (pos[1] - self.center[1])/(pos[0] - self.center[0]).atan()
    }

    fn comp_pos(&self, theta: f64) -> Vector2<f64> {
        self.center + self.radius * Vector2::new(theta.cos(), (self.q * theta).sin())
    }

    fn comp_tangent(&self, theta: f64) -> UnitVector2<f64> {
        let tangent = Vector2::new(-theta.sin(), self.q * theta.cos());
        UnitVector2::new_normalize(tangent)
    }
}