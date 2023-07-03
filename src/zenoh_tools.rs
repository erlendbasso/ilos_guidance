use crate::{ilos::ILOS, path::Path};

use zenoh::{prelude::r#async::*, publication::Publisher};
// use zenoh::config::Config;
use cdr::{CdrLe, Infinite};
use serde_derive::{Deserialize, Serialize};
use std::fmt;
use std::sync::{Arc, Mutex};

extern crate nalgebra as na;
use na::Vector2;

pub async fn ilos_timer(
    session: Arc<Session>,
    topic_name: String,
    arc_pos: Arc<Mutex<Option<Vector2<f64>>>>,
    // ilos: Arc<Mutex<ILOS>>,
    ilos: &mut ILOS,
    mut path: impl Path,
    dt: f64,
) {
    let publisher = session.declare_publisher(topic_name).res().await.unwrap();

    let mut timer = tokio::time::interval(tokio::time::Duration::from_secs_f64(dt));
    loop {
        timer.tick().await;

        let pos = arc_pos.lock().unwrap().unwrap();
        let theta = path.comp_theta(&pos);
        let pos_desired = path.comp_pos(theta);
        let tau_desired = path.comp_tangent(theta);

        ilos.update(&pos, &pos_desired, &tau_desired, dt);
        let (yaw, yaw_rate) = ilos.get_references();

        publish_ilos_message(&publisher, yaw, yaw_rate).await;
    }
}

pub async fn publish_ilos_message(publisher: &Publisher<'_>, yaw: f64, yaw_rate: f64) {
    let t_now = std::time::SystemTime::now();
    let since_epoch = t_now.duration_since(std::time::UNIX_EPOCH).unwrap();

    let header = Header {
        stamp: Time {
            sec: since_epoch.as_secs() as i32,
            nanosec: since_epoch.subsec_nanos(),
        },
        frame_id: "".to_string(),
    };

    let ilos_msg = ILOSMessage {
        header,
        yaw,
        yaw_rate,
    };

    let encoded = cdr::serialize::<_, _, CdrLe>(&ilos_msg, Infinite).unwrap();
    if let Err(e) = publisher.put(encoded).res().await {
        println!("Error writing {}: {}", publisher.key_expr().as_str(), e);
    }
}

pub async fn position_subscriber(
    session: Arc<Session>,
    topic_name: String,
    arc_pos: Arc<Mutex<Option<Vector2<f64>>>>,
) {
    let subscriber = session.declare_subscriber(topic_name).res().await.unwrap();

    while let Ok(sample) = subscriber.recv_async().await {
        match cdr::deserialize_from::<_, Odometry, _>(
            sample.value.payload.reader(),
            cdr::size::Infinite,
        ) {
            Ok(odom) => {
                let pos = Vector2::new(odom.pose.pose.position.x, odom.pose.pose.position.y);
                let mut pos_ref = arc_pos.lock().unwrap();
                *pos_ref = Some(pos);
            }
            // Err(e) => log::warn!("Error decoding Log: {}", e),
            Err(e) => println!("Error decoding Odometry msg: {}", e),
        }
    }
}

#[derive(Deserialize, Serialize, PartialEq, Debug)]
struct ROSVector3 {
    x: f64,
    y: f64,
    z: f64,
}

#[derive(Deserialize, Serialize, PartialEq, Debug)]
struct Point {
    x: f64,
    y: f64,
    z: f64,
}

#[derive(Deserialize, Serialize, PartialEq, Debug)]
struct Twist {
    linear: ROSVector3,
    angular: ROSVector3,
}

#[derive(Deserialize, Serialize, PartialEq, Debug)]
struct Covariance6 {
    col1: [f64; 6],
    col2: [f64; 6],
    col3: [f64; 6],
    col4: [f64; 6],
    col5: [f64; 6],
    col6: [f64; 6],
}

#[derive(Deserialize, Serialize, PartialEq, Debug)]
struct TwistWithCovariance {
    twist: Twist,
    // covariance: Vec<f64>,
    covariance: Covariance6,
}

#[derive(Deserialize, Serialize, PartialEq, Debug)]
struct Quaternion {
    x: f64,
    y: f64,
    z: f64,
    w: f64,
}

#[derive(Deserialize, Serialize, PartialEq, Debug)]
struct Pose {
    position: Point,
    orientation: Quaternion,
}

#[derive(Deserialize, Serialize, PartialEq, Debug)]
struct PoseWithCovariance {
    pose: Pose,
    covariance: Covariance6,
}

#[derive(Serialize, Deserialize, PartialEq)]
struct Time {
    sec: i32,
    nanosec: u32,
}

#[derive(Serialize, Deserialize, PartialEq)]
struct Header {
    stamp: Time,
    frame_id: String,
}

#[derive(Serialize, Deserialize, PartialEq)]
struct ILOSMessage {
    header: Header,
    yaw: f64,
    yaw_rate: f64,
}

#[derive(Deserialize, PartialEq)]
struct Odometry {
    header: Header,
    child_frame_id: String,
    pose: PoseWithCovariance,
    twist: TwistWithCovariance,
}

impl fmt::Display for Odometry {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "[{}.{}] pos: ({}, {}, {}) quat: (w: {}, x: {}, y: {}, z: {}) vel: ({}, {}, {}) ang_vel: ({}, {}, {})",
            self.header.stamp.sec,
            self.header.stamp.nanosec,
            self.pose.pose.position.x,
            self.pose.pose.position.y,
            self.pose.pose.position.z,
            self.pose.pose.orientation.w,
            self.pose.pose.orientation.x,
            self.pose.pose.orientation.y,
            self.pose.pose.orientation.z,
            self.twist.twist.linear.x,
            self.twist.twist.linear.y,
            self.twist.twist.linear.z,
            self.twist.twist.angular.x,
            self.twist.twist.angular.y,
            self.twist.twist.angular.z,
        )
    }
}
