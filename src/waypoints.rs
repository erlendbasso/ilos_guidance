extern crate nalgebra as na;

use crate::circle::Circle;
use crate::line::Line;
use crate::path::Path;
use na::{Matrix2, UnitVector2, Vector2};

#[derive(Clone, Debug)]
enum PathSegment {
    Line,
    Circle,
}

pub struct WaypointPath {
    current_waypoint: usize,
    current_path_segment: PathSegment,
    lines: Vec<Line>,
    circles: Vec<Circle>,
    theta_circ_min: Vec<f64>,
    theta_circ_max: Vec<f64>,
    theta_line_min: Vec<f64>,
    theta_line_max: Vec<f64>,
}

#[allow(non_snake_case)]
impl WaypointPath {
    pub fn new(waypoints: Vec<Vector2<f64>>, circle_radius: f64) -> WaypointPath {
        let mut lines = Vec::new();
        let mut circles = Vec::new();
        let mut theta_circ_min = Vec::new();
        let mut theta_circ_max = Vec::new();
        let mut theta_line_min = Vec::new();
        let mut theta_line_max = Vec::new();

        let S = Matrix2::new(0.0, -1.0, 1.0, 0.0);

        theta_line_min.push(-f64::INFINITY);

        for i in 1..waypoints.len() {
            let line: Line = Line::new(waypoints[i - 1], waypoints[i]);
            lines.push(line);
        }
        let mut v = (waypoints[1] - waypoints[0]).normalize();

        for i in 0..waypoints.len() - 2 {
            let v_next = (waypoints[i + 2] - waypoints[i + 1]).normalize();
            let d = circle_radius * ((S * v).dot(&v_next)).abs() / (1.0 + v.dot(&v_next));
            let q = (S * v).dot(&v_next).signum();
            let clockwise = q < 0.0;

            let theta_max_line = lines[i].comp_theta(&(waypoints[i + 1] - d * v));
            let theta_min_line = lines[i + 1].comp_theta(&(waypoints[i + 1] + d * v_next));
            theta_line_min.push(theta_min_line);

            theta_line_max.push(theta_max_line);

            let pos_circle = waypoints[i + 1] - d * v + q * circle_radius * S * v;
            let circle: Circle = Circle::new(circle_radius, pos_circle, clockwise);
            circles.push(circle);

            let theta_min_circ = circles[i].comp_theta(&(waypoints[i + 1] - d * v));
            let theta_max_circ = circles[i].comp_theta(&(waypoints[i + 1] + d * v_next));

            theta_circ_min.push(theta_min_circ);
            theta_circ_max.push(theta_max_circ);

            v = v_next;
        }

        theta_line_max.push(f64::INFINITY);

        WaypointPath {
            // waypoints,
            current_waypoint: 1,
            current_path_segment: PathSegment::Line,
            lines,
            circles,
            theta_circ_min,
            theta_circ_max,
            theta_line_min,
            theta_line_max,
        }
    }
}

impl Path for WaypointPath {
    fn comp_theta(&mut self, pos: &Vector2<f64>) -> f64 {
        match self.current_path_segment {
            PathSegment::Line => {
                let theta = self.lines[self.current_waypoint - 1].comp_theta(pos);
                if theta > self.theta_line_max[self.current_waypoint - 1] {
                    self.current_path_segment = PathSegment::Circle;
                } else if theta < self.theta_line_min[self.current_waypoint - 1] {
                    self.current_waypoint -= 1;
                    self.current_path_segment = PathSegment::Circle;
                }
            }
            PathSegment::Circle => {
                let theta = self.circles[self.current_waypoint - 1].comp_theta(pos);
                if theta > self.theta_circ_max[self.current_waypoint - 1] {
                    self.current_waypoint += 1;
                    self.current_path_segment = PathSegment::Line;
                } else if theta < self.theta_circ_min[self.current_waypoint - 1] {
                    self.current_path_segment = PathSegment::Line;
                }
            }
        };

        let theta = match self.current_path_segment {
            PathSegment::Line => self.lines[self.current_waypoint - 1].comp_theta(pos),
            PathSegment::Circle => self.circles[self.current_waypoint - 1].comp_theta(pos),
        };
        theta
    }

    fn comp_pos(&self, theta: f64) -> Vector2<f64> {
        let pos = match self.current_path_segment {
            PathSegment::Line => self.lines[self.current_waypoint - 1].comp_pos(theta),
            PathSegment::Circle => self.circles[self.current_waypoint - 1].comp_pos(theta),
        };
        pos
    }

    fn comp_tangent(&self, theta: f64) -> UnitVector2<f64> {
        let tau = match self.current_path_segment {
            PathSegment::Line => self.lines[self.current_waypoint - 1].comp_tangent(theta),
            PathSegment::Circle => self.circles[self.current_waypoint - 1].comp_tangent(theta),
        };
        tau
    }
}
