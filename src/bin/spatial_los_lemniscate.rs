use ilos_guidance::{
    los::LOS,
    paths::spatial_lemniscate::{BgdParameters, SpatialLemniscate},
    zenoh_tools::*,
};

use std::sync::{Arc, Mutex};

extern crate nalgebra as na;
use na::Vector3;

use clap::Parser;
use serde::Deserialize;
use std::{fs::File, io::BufReader};

#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    /// Optional YAML config file path
    #[arg(long)]
    config: Option<String>,
    /// Odometry subscriber topic name
    #[arg(short, long, default_value = "snake/odom")]
    topic: String,
    /// Output spatial LOS message topic name
    #[arg(long, default_value = "snake/spatial_los")]
    topic_out: String,
    /// Frequency of the controller
    #[arg(short, long, default_value_t = 100)]
    freq: u64,
    /// Height of the lemniscate (x half-width)
    #[arg(long, default_value_t = 10.0)]
    height: f64,
    /// Width of the lemniscate (y half-width)
    #[arg(short, long, default_value_t = 15.0)]
    width: f64,
    /// Amplitude of the sinusoidal z-component
    #[arg(long, default_value_t = 2.0)]
    amplitude: f64,
    /// Center of the lemniscate
    #[arg(short, long, default_values_t = [0.0, 0.0, 0.0])]
    center: Vec<f64>,
    /// Initial value for theta
    #[arg(long, default_value_t = 0.0)]
    theta_0: f64,
    /// LOS lookahead distance [m]
    #[arg(short, long, default_value_t = 5.0)]
    delta: f64,
    /// Max value of step size in BGD (initial guess)
    #[arg(long, default_value_t = 0.1)]
    s_bar: f64,
    /// How much decrease is needed to accept next step in BGD
    #[arg(long, default_value_t = 0.1)]
    sigma: f64,
    /// Scaling factor to decrease the step size in BGD
    #[arg(short, long, default_value_t = 0.1)]
    mu: f64,
}

#[derive(Debug, Deserialize)]
struct SpatialLemniscateConfig {
    topic: String,
    topic_out: String,
    freq: u64,
    height: f64,
    width: f64,
    amplitude: f64,
    center: [f64; 3],
    theta_0: f64,
    delta: f64,
    s_bar: f64,
    sigma: f64,
    mu: f64,
}

fn load_config(path: &str) -> Result<SpatialLemniscateConfig, Box<dyn std::error::Error>> {
    let file = File::open(path)?;
    let reader = BufReader::new(file);
    let cfg: SpatialLemniscateConfig = serde_yaml::from_reader(reader)?;
    Ok(cfg)
}

#[tokio::main]
async fn main() {
    let args = Args::parse();

    // Load from config if provided; otherwise use CLI values
    let (
        topic_name,
        output_topic_name,
        freq,
        lemni_height,
        lemni_width,
        lemni_amplitude,
        lemni_center,
        theta_0,
        delta,
        s_bar,
        sigma,
        mu,
    ) = if let Some(cfg_path) = &args.config {
        let cfg = load_config(cfg_path).expect("Failed to load config file");
        (
            cfg.topic,
            cfg.topic_out,
            cfg.freq,
            cfg.height,
            cfg.width,
            cfg.amplitude,
            Vector3::new(cfg.center[0], cfg.center[1], cfg.center[2]),
            cfg.theta_0,
            cfg.delta,
            cfg.s_bar,
            cfg.sigma,
            cfg.mu,
        )
    } else {
        (
            args.topic,
            args.topic_out,
            args.freq,
            args.height,
            args.width,
            args.amplitude,
            Vector3::new(args.center[0], args.center[1], args.center[2]),
            args.theta_0,
            args.delta,
            args.s_bar,
            args.sigma,
            args.mu,
        )
    };

    println!("Subscribing to topic: {}", topic_name);
    println!("Controller frequency: {}", freq);
    println!("Controller period: {}", 1.0 / (freq as f64));
    println!("Lookahead distance: {}", delta);

    let los = LOS::<3>::new(delta);
    let arc_los = Arc::new(Mutex::new(los));

    let session = zenoh::open(zenoh::Config::default()).await.unwrap();

    let pos_measured: Option<Vector3<f64>> = None;
    let arc_pos = Arc::new(Mutex::new(pos_measured));
    let pos_measured = arc_pos.clone();

    let an_session = session.clone();

    tokio::spawn(async move {
        spatial_position_subscriber(an_session, topic_name, pos_measured).await;
    });

    // BGD parameters
    let bgd_params = BgdParameters::new(s_bar, sigma, mu);
    let lemniscate = SpatialLemniscate::new(
        lemni_height,
        lemni_width,
        lemni_amplitude,
        lemni_center,
        bgd_params,
    );

    let an_session = session.clone();
    let an_los = arc_los.clone();
    let pos_measured = arc_pos.clone();
    let dt = 1.0 / (freq as f64);
    println!("dt: {}", dt);

    tokio::spawn(async move {
        spatial_los_timer_lemniscate(
            an_session,
            output_topic_name,
            pos_measured,
            an_los,
            lemniscate,
            theta_0,
            dt,
        )
        .await;
    })
    .await
    .unwrap();
}
