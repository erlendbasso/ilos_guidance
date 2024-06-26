extern crate nalgebra as na;
use na::{UnitVector2, Vector2};

pub trait Path {
    fn comp_pos(&self, theta: f64) -> Vector2<f64>;
    fn comp_tangent(&self, theta: f64) -> UnitVector2<f64>;
    fn comp_theta(&mut self, pos: &Vector2<f64>) -> f64;
}
