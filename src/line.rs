extern crate nalgebra as na;

use na::{UnitVector2, Vector2};

use crate::path::Path;

#[derive(Clone)]
pub struct Line {
    angle: f64,
    pos_0: Vector2<f64>,
}

impl Line {
    pub fn new(pos_0: Vector2<f64>, pos_1: Vector2<f64>) -> Line {
        let angle = (pos_1[1] - pos_0[1]).atan2(pos_1[0] - pos_0[0]);
        // println!("angle: {}", angle);
        Line { angle, pos_0 }
    }
}

impl Path for Line {
    fn comp_theta(&self, pos: &Vector2<f64>) -> f64 {
        // println!("pos: {}, pos_0: {}", pos, self.pos_0);
        (pos - self.pos_0).dot(&(Vector2::new(self.angle.cos(), self.angle.sin())))
    }

    fn comp_pos(&self, theta: f64) -> Vector2<f64> {
        self.pos_0 + Vector2::new(self.angle.cos(), self.angle.sin()) * theta
    }

    #[allow(unused_variables)]
    fn comp_tangent(&self, theta: f64) -> UnitVector2<f64> {
        let tangent = Vector2::new(self.angle.cos(), self.angle.sin());
        UnitVector2::new_normalize(tangent)
    }
}
