use ilos_guidance::{
    ilos::ILOS,
    alos::ALOS,
    paths::lemniscate::{BgdParameters, Lemniscate},
    zenoh_tools::*,
};

// use serde_derive::{Deserialize, Serialize};
// use std::fmt;
use std::sync::{Arc, Mutex};
use core::f64::consts::PI;

extern crate nalgebra as na;
use na::Vector2;

use clap::Parser;

#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    /// Odometry subscriber topic name
    #[arg(short, long, default_value = "blueboat/odom")]
    topic: String,
    /// Output ILOS message topic name
    #[arg(short, long, default_value = "blueboat/yaw_reference")]
    topic_out: String,
    /// Frequency of the controller
    #[arg(short, long, default_value_t = 100)]
    freq: u64,
    /// Height of the lemniscate
    #[arg(short, long, default_value_t = 10.0)]
    height: f64,
    /// Width of the lemniscate
    #[arg(short, long, default_value_t = 15.0)]
    width: f64,
    /// Center of the lemniscate
    #[arg(short, long, default_values_t = [0.0, 0.0])]
    center: Vec<f64>,
    /// Initial value for theta
    #[arg(short, long, default_value_t = 0.0)]
    theta_0: f64,
    /// ILOS proportional gain
    #[arg(short, long, default_value_t = 1.0)]
    kp: f64,
    /// ILOS integral gain
    #[arg(short, long, default_value_t = 0.01)]
    ki: f64,
    /// ALOS saturation limit [deg]
    #[arg(short, long, default_value_t = 5.0)]
    saturation_limit: f64,
    /// Max value of step size in BGD (initial guess)
    #[arg(short, long, default_value_t = 0.1)]
    s_bar: f64,
    /// How much decrease is needed to accept next step in BGD
    #[arg(short, long, default_value_t = 0.1)]
    sigma: f64,
    /// Scaling factor to decrease the step size in BGD
    #[arg(short, long, default_value_t = 0.1)]
    mu: f64,
}

#[tokio::main]
async fn main() {
    let args = Args::parse();
    let topic_name = args.topic;
    let freq = args.freq;
    let output_topic_name = args.topic_out;
    let lemni_height = args.height;
    let lemni_width = args.width;
    let lemni_center = Vector2::new(args.center[0], args.center[1]);
    let kp = args.kp;
    let ki = args.ki;
    let saturation_limit = PI / 180.0 *  args.saturation_limit;
    let theta_0 = args.theta_0;
    let s_bar = args.s_bar;
    let sigma = args.sigma;
    let mu = args.mu;

    let param_topic = "ilos/params".to_string();

    println!("Subscribing to topic: {}", topic_name);
    println!("Controller frequency: {}", freq);
    println!("Controller period: {}", 1 / freq);

    let alos = ALOS::new(kp, ki, saturation_limit);
    let arc_alos = Arc::new(Mutex::new(alos));

    let session = zenoh::open(zenoh::Config::default()).await.unwrap();

    let pos_measured: Option<Vector2<f64>> = None;
    let arc_pos = Arc::new(Mutex::new(pos_measured));
    let pos_measured = arc_pos.clone();

    let an_session = session.clone();

    tokio::spawn(async move {
        position_subscriber(an_session, topic_name, pos_measured).await;
    });

    let an_session = session.clone();
    let an_alos = arc_alos.clone();

    // tokio::spawn(async move {
    //     update_ilos_parameters(an_session, param_topic, an_alos).await;
    // });

    // let circle = Circle::new(radius, center, clockwise);

    // let circle = Circle::new(circle_radius, circle_center, false);
    // BGD parameters

    let bgd_params = BgdParameters::new(s_bar, sigma, mu);
    let lemniscate = Lemniscate::new(lemni_height, lemni_width, lemni_center, bgd_params);

    let an_session = session.clone();
    let an_alos = arc_alos.clone();
    let pos_measured = arc_pos.clone();
    let dt = 1.0 / (freq as f64);
    println!("dt: {}", dt);

    tokio::spawn(async move {
        alos_timer_lemniscate(
            an_session,
            output_topic_name,
            pos_measured,
            an_alos,
            lemniscate,
            theta_0,
            dt,
        )
        .await;
    })
    .await
    .unwrap();
}
