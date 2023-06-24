extern crate nalgebra as na;

use na::{Vector2, Matrix2, UnitVector2};
use crate::line::Line;
use crate::circle::{Circle, self};
use crate::path::Path;

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
            let line : Line = Line::new(waypoints[i - 1], waypoints[i]);
            lines.push(line);
        }
        let mut v = (waypoints[1] - waypoints[0]).normalize();
        
        for i in 0..waypoints.len() - 2 {
            let v_next = (waypoints[i+2] - waypoints[i+1]).normalize();
            let d = circle_radius * ((S * v).dot(&v_next)).abs() / (1.0 + v.dot(&v_next)) ;
            let q = (S * v).dot(&v_next).signum();
            let clockwise = q < 0.0;
            // if clockwise {
            //     println!("clockwise");
            // } else {
            //     println!("counter-clockwise");
            // }


            let theta_max_line = lines[i].comp_theta(&(waypoints[i + 1] - d * v));
            let theta_min_line = lines[i+1].comp_theta(&(waypoints[i + 1] + d * v_next));
            theta_line_min.push(theta_min_line);

            theta_line_max.push(theta_max_line);


            let pos_circle = waypoints[i + 1] - d * v + q * circle_radius * S * v;
            let circle : Circle = Circle::new(circle_radius, pos_circle, clockwise);
            circles.push(circle);

            let theta_min_circ = circles[i].comp_theta(&(waypoints[i + 1] - d * v));
            let theta_max_circ = circles[i].comp_theta(&(waypoints[i + 1] + d * v_next));

            theta_circ_min.push(theta_min_circ);
            theta_circ_max.push(theta_max_circ);

            v = v_next;
        }

        theta_line_max.push(f64::INFINITY);

        // println!("theta_circ_min: {:?}", theta_circ_min);
        // println!("theta_circ_max: {:?}", theta_circ_max);
        // println!("theta_line_min: {:?}", theta_line_min);
        // println!("theta_line_max: {:?}", theta_line_max);
        // println!("waypoints: {:?}", waypoints);


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

    pub fn comp_pos_tangent_refs(&mut self, pos: &Vector2<f64>) -> (Vector2<f64>, UnitVector2<f64>) {
        match self.current_path_segment {
            PathSegment::Line => {
                // println!("current_waypoint: {}", pos);
                let theta = self.lines[self.current_waypoint - 1].comp_theta(pos);
                if theta > self.theta_line_max[self.current_waypoint - 1] {
                    // self.current_waypoint += 1;
                    self.current_path_segment = PathSegment::Circle;
                }
                else if theta < self.theta_line_min[self.current_waypoint - 1] {
                    self.current_waypoint -= 1;
                    // println!("GOING BACK TO PREVIOUS WAYPOINT");
                    self.current_path_segment = PathSegment::Circle;
                }
            },
            PathSegment::Circle => {
                let theta = self.circles[self.current_waypoint - 1].comp_theta(pos);
                // println!("current waypoint: {}", self.current_waypoint);
                // println!("theta: {}", theta);
                if theta > self.theta_circ_max[self.current_waypoint - 1]  {
                    self.current_waypoint += 1;
                    self.current_path_segment = PathSegment::Line;
                }
                else if theta < self.theta_circ_min[self.current_waypoint - 1] {
                    self.current_path_segment = PathSegment::Line;
                }
            },
        };

        let theta = match self.current_path_segment {
            PathSegment::Line => self.lines[self.current_waypoint - 1].comp_theta(pos),
            PathSegment::Circle => self.circles[self.current_waypoint - 1].comp_theta(pos),
        };

        let pos = match self.current_path_segment {
            PathSegment::Line => self.lines[self.current_waypoint - 1].comp_pos(theta),
            PathSegment::Circle => self.circles[self.current_waypoint - 1].comp_pos(theta),
        };

        let tau = match self.current_path_segment {
            PathSegment::Line => self.lines[self.current_waypoint - 1].comp_tangent(theta),
            PathSegment::Circle => self.circles[self.current_waypoint - 1].comp_tangent(theta),
        };

        // println!("tau: {:?}", tau);
        (pos, tau)
    }


}

#[cfg(test)]
mod tests{
    use super::*;

    #[test]
    fn test_path() {
        let mut wtr = csv::Writer::from_path("ilos_path.csv").unwrap();
        let radius = 0.1;
        let mut pos = Vector2::new(0.0, 0.0);
        let waypoints = vec![
            pos,
            Vector2::new(1.0, 1.0),
            Vector2::new(2.0, 0.0),
            Vector2::new(7.0, 15.0),
            Vector2::new(0.0, 0.0),
            // Vector2::new(1.0, 1.0),
            // Vector2::new(1.0, 2.0),
            // Vector2::new(7.0, 15.0),
            // Vector2::new(0.0, 0.0),
        ];
        let mut wp_path = WaypointPath::new(waypoints, radius);

        wtr.write_record(&[
            "pos_x","pos_y","tau_x","tau_y",
        ]).unwrap();
        
        // println!("pos: {}", pos);
        let (mut pos, mut tau) = wp_path.comp_pos_tangent_refs(&pos);
        println!("pos: {}", pos);

        let mut pos_m = pos;

        wtr.write_record(&[
            &pos[0].to_string(),
            &pos[1].to_string(),
            &tau[0].to_string(),
            &tau[1].to_string(),
        ]).unwrap();

        for _i in 0..100000 {
            pos_m += 0.0005 * tau.into_inner();
            (pos, tau) = wp_path.comp_pos_tangent_refs(&pos_m);
            wtr.write_record(&[
                &pos[0].to_string(),
                &pos[1].to_string(),
                &tau[0].to_string(),
                &tau[1].to_string(),
            ]).unwrap();
            
        }
        wtr.flush();
    }
}
