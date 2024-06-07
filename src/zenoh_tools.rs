use crate::{ilos::ILOS, paths::path::Path};

use cdr::{CdrLe, Infinite};
use serde_derive::{Deserialize, Serialize};
use std::fmt;
use std::sync::{Arc, Mutex};
use zenoh::{prelude::r#async::*, publication::Publisher};

use tokio::select;
extern crate nalgebra as na;
use na::Vector2;

pub async fn ilos_timer(
    session: Arc<Session>,
    topic_name: String,
    arc_pos: Arc<Mutex<Option<Vector2<f64>>>>,
    ilos: Arc<Mutex<ILOS>>,
    mut path: impl Path,
    dt: f64,
) {
    let publisher = session.declare_publisher(topic_name).res().await.unwrap();

    let mut timer = tokio::time::interval(tokio::time::Duration::from_secs_f64(dt));
    loop {
        timer.tick().await;

        let pos = {
            let pos_guard = arc_pos.lock().unwrap();
            if *pos_guard == None {
                continue;
            }
            pos_guard.unwrap()
        };
        let theta = path.comp_theta(&pos);
        let pos_desired = path.comp_pos(theta);
        let tau_desired = path.comp_tangent(theta);

        let (yaw, yaw_rate) = {
            let mut ilos = ilos.lock().unwrap();
            ilos.update(&pos, &pos_desired, &tau_desired, dt);
            ilos.get_references()
        };
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
            Err(e) => println!("Error decoding Odometry msg: {}", e),
        }
    }
}

pub async fn update_ilos_parameters(
    session: Arc<Session>,
    key_expr: String,
    ilos: Arc<Mutex<ILOS>>,
) {
    let key_expr = KeyExpr::try_from(key_expr).unwrap();

    let (kp, ki) = {
        let ilos = ilos.lock().unwrap();
        ilos.get_gains()
    };
    let mut ilos_params = ILOSParameters {
        proportional_gain: kp,
        integral_gain: ki,
    };

    println!("Declaring Parameter Subscriber on '{key_expr}'...");
    let subscriber = session.declare_subscriber(&key_expr).res().await.unwrap();

    println!("Declaring Parameter Queryable on '{key_expr}'...");
    let queryable = session
        .declare_queryable(&key_expr)
        // .complete(complete)
        .res()
        .await
        .unwrap();

    loop {
        select!(
            sample = subscriber.recv_async() => {
                let sample = sample.unwrap();
                let data = sample.value.payload.contiguous().into_owned();
                match serde_json::from_str(String::from_utf8(data).unwrap().as_str()) {
                    Ok(params) => {
                        ilos_params = params;
                        let mut ilos = ilos.lock().unwrap();
                        println!(">> [Subscriber] Received ILOS Parameters: {:?}", ilos_params);
                        ilos.set_gains(ilos_params.proportional_gain, ilos_params.integral_gain);
                    }
                    Err(e) => println!("Error decoding ILOS parameter msg: {}", e),
                }
            },

            query = queryable.recv_async() => {
                let query = query.unwrap();
                println!(">> [Queryable ] Received Query '{}'", query.selector());

                let encoded = serde_json::to_string(&ilos_params).unwrap().into_bytes();

                let mut value = Value::empty();
                value.encoding = Encoding::Exact(KnownEncoding::AppJson);
                value.payload = encoded.into();

                let sample = Sample::new(key_expr.clone(), value);
                query.reply(Ok(sample)).res().await.unwrap();
            }
        );
    }
}

#[derive(Serialize, Deserialize, PartialEq, Debug, Clone)]
struct ILOSParameters {
    proportional_gain: f64,
    integral_gain: f64,
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
