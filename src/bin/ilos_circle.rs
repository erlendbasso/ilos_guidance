use ilos_guidance::{paths::circle::Circle, ilos::ILOS, zenoh_tools::*};

use zenoh::prelude::r#async::*;
// use serde_derive::{Deserialize, Serialize};
// use std::fmt;
use std::sync::{Arc, Mutex};

extern crate nalgebra as na;
use na::Vector2;

use clap::Parser;

#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    /// Odometry subscriber topic name
    #[arg(short, long, default_value = "rt/odom")]
    topic: String,
    /// Output ILOS message topic name
    #[arg(short, long, default_value = "rt/yaw_refs")]
    topic_out: String,
    /// Frequency of the controller
    #[arg(short, long, default_value_t = 100)]
    freq: u64,
    /// Radius of the circle
    #[arg(short, long, default_value_t = 3.0)]
    radius: f64,
    /// Center of the circle
    #[arg(short, long, default_values_t = [0.0, 0.0])]
    center: Vec<f64>,
    /// ILOS proportional gain
    #[arg(short, long, default_value_t = 1.0)]
    kp: f64,
    /// ILOS integral gain
    #[arg(short, long, default_value_t = 0.01)]
    ki: f64,
}

#[tokio::main]
async fn main() {
    let args = Args::parse();
    let topic_name = args.topic;
    let freq = args.freq;
    let output_topic_name = args.topic_out;
    let circle_radius = args.radius;
    let circle_center = Vector2::new(args.center[0], args.center[1]);
    let kp = args.kp;
    let ki = args.ki;

    let param_topic = "ilos/params".to_string();

    println!("Subscribing to topic: {}", topic_name);
    println!("Controller frequency: {}", freq);
    println!("Controller period: {}", 1 / freq);

    let ilos = ILOS::new(kp, ki);
    let arc_ilos = Arc::new(Mutex::new(ilos));

    let session = zenoh::open(config::default())
        .res()
        .await
        .unwrap()
        .into_arc();

    let pos_measured: Option<Vector2<f64>> = None;
    let arc_pos = Arc::new(Mutex::new(pos_measured));
    let pos_measured = arc_pos.clone();

    let an_session = session.clone();

    tokio::spawn(async move {
        position_subscriber(an_session, topic_name, pos_measured).await;
    });

    let an_session = session.clone();
    let an_ilos = arc_ilos.clone();

    tokio::spawn(async move {
        update_ilos_parameters(an_session, param_topic, an_ilos).await;
    });

    // let circle = Circle::new(radius, center, clockwise);

    let circle = Circle::new(circle_radius, circle_center, false);

    let an_session = session.clone();
    let an_ilos = arc_ilos.clone();
    let pos_measured = arc_pos.clone();
    let dt = 1.0 / (freq as f64);
    println!("dt: {}", dt);

    tokio::spawn(async move {
        ilos_timer(
            an_session,
            output_topic_name,
            pos_measured,
            an_ilos,
            circle,
            dt,
        )
        .await;
    })
    .await
    .unwrap();
}
