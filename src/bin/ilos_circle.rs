use ilos_guidance::{circle::Circle, ilos::ILOS};

use zenoh::prelude::r#async::*;
// use zenoh::config::Config;
use cdr::{CdrLe, Infinite};
use serde_derive::{Deserialize, Serialize};
use std::fmt;
use std::sync::Arc;

extern crate nalgebra as na;
use na::{Matrix6, SMatrix, Vector3};

use clap::Parser;

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

#[derive(Deserialize, PartialEq)]
struct Time {
    sec: i32,
    nanosec: u32,
}

#[derive(Deserialize, PartialEq)]
struct Header {
    stamp: Time,
    frame_id: String,
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

#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    /// Odometry subscriber topic name
    #[arg(short, long, default_value = "rt/odom")]
    topic: String,
    // /// Number of times to greet
    // #[arg(short, long, default_value_t = 1)]
    // count: u8,
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = Args::parse();
    let topic_name = args.topic;
    println!("Subscribing to topic: {}", topic_name);

    let session = zenoh::open(config::default())
        .res()
        .await
        .unwrap()
        .into_arc();

    let an_session = session.clone();

    tokio::spawn(async move {
        position_subscriber(an_session, topic_name).await;
    })
    .await
    .unwrap();

    Ok(())
}

async fn position_subscriber(session: Arc<Session>, topic_name: String) {
    let subscriber = session.declare_subscriber(topic_name).res().await.unwrap();

    while let Ok(sample) = subscriber.recv_async().await {
        match cdr::deserialize_from::<_, Odometry, _>(
            sample.value.payload.reader(),
            cdr::size::Infinite,
        ) {
            Ok(odometry) => {
                println!("{}", odometry);
                let pos = Vector3::new(
                    odometry.pose.pose.position.x,
                    odometry.pose.pose.position.y,
                    odometry.pose.pose.position.z,
                );
            }
            // Err(e) => log::warn!("Error decoding Log: {}", e),
            Err(e) => println!("Error decoding Odometry: {}", e),
        }
    }
}
