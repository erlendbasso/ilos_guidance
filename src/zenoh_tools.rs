use crate::{
    alos::ALOS, ilos::ILOS, los::LOS, paths::lemniscate::Lemniscate, paths::path::Path,
    paths::spatial_lemniscate::SpatialLemniscate,
};

use cdr::{CdrLe, Infinite};
use serde_derive::{Deserialize, Serialize};
use std::fmt;
use std::sync::{Arc, Mutex};
// use zenoh::publication::Publisher;
use zenoh::bytes::Encoding;
use zenoh::key_expr::KeyExpr;
// use zenoh::prelude::*;
use zenoh::pubsub::Publisher;
use zenoh::Session;

use tokio::select;
extern crate nalgebra as na;
use na::{Unit, UnitVector3, Vector2, Vector3};

pub async fn ilos_timer(
    session: Session,
    topic_name: String,
    arc_pos: Arc<Mutex<Option<Vector2<f64>>>>,
    ilos: Arc<Mutex<ILOS>>,
    mut path: impl Path,
    dt: f64,
) {
    let publisher = session.declare_publisher(topic_name).await.unwrap();
    let pub_desired_pos_theta = session
        .declare_publisher("blueboat/desired_pos")
        .await
        .unwrap();

    let mut timer = tokio::time::interval(tokio::time::Duration::from_secs_f64(dt));
    loop {
        timer.tick().await;

        let pos = {
            let pos_guard = arc_pos.lock().unwrap();
            if (*pos_guard).is_none() {
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
        publish_desired_pos_theta(&pub_desired_pos_theta, &pos_desired, theta).await;
    }
}

pub async fn ilos_timer_lemniscate(
    session: Session,
    topic_name: String,
    arc_pos: Arc<Mutex<Option<Vector2<f64>>>>,
    ilos: Arc<Mutex<ILOS>>,
    path: Lemniscate,
    theta_0: f64,
    dt: f64,
) {
    let publisher = session.declare_publisher(topic_name).await.unwrap();
    let pub_desired_pos_theta = session
        .declare_publisher("blueboat/desired_pos")
        .await
        .unwrap();
    let mut theta_prev = theta_0;

    let mut timer = tokio::time::interval(tokio::time::Duration::from_secs_f64(dt));
    loop {
        timer.tick().await;

        let pos = {
            let pos_guard = arc_pos.lock().unwrap();
            if (*pos_guard).is_none() {
                continue;
            }
            pos_guard.unwrap()
        };
        let theta = path.comp_theta_bgd(&pos, theta_prev);
        let pos_desired = path.comp_pos(theta);
        let tau_desired = path.comp_tangent(theta);

        let (yaw, yaw_rate) = {
            let mut ilos = ilos.lock().unwrap();
            ilos.update(&pos, &pos_desired, &tau_desired, dt);
            ilos.get_references()
        };
        publish_ilos_message(&publisher, yaw, yaw_rate).await;
        publish_desired_pos_theta(&pub_desired_pos_theta, &pos_desired, theta).await;

        theta_prev = theta;
    }
}

pub async fn spatial_los_timer_lemniscate(
    session: Session,
    topic_name: String,
    arc_pos: Arc<Mutex<Option<Vector3<f64>>>>,
    los: Arc<Mutex<LOS<3>>>,
    path: SpatialLemniscate,
    theta_0: f64,
    dt: f64,
) {
    let publisher = session.declare_publisher(topic_name).await.unwrap();
    let pub_desired_pos = session
        .declare_publisher("snake/desired_pos")
        .await
        .unwrap();
    let mut theta_prev = theta_0;

    let mut timer = tokio::time::interval(tokio::time::Duration::from_secs_f64(dt));
    loop {
        timer.tick().await;

        let pos = {
            let pos_guard = arc_pos.lock().unwrap();
            if (*pos_guard).is_none() {
                continue;
            }
            pos_guard.unwrap()
        };
        let theta = path.comp_theta_bgd(&pos, theta_prev);
        let pos_desired = path.comp_pos(theta);
        let tau_desired = path.comp_tangent(theta);

        let mu = {
            let mut los = los.lock().unwrap();
            los.update(&pos, &pos_desired, &tau_desired)
        };
        publish_spatial_los_message(&publisher, &mu).await;
        publish_desired_spatial_pos(&pub_desired_pos, &pos_desired).await;

        theta_prev = theta;
    }
}

pub async fn alos_timer_lemniscate(
    session: Session,
    topic_name: String,
    arc_pos: Arc<Mutex<Option<Vector2<f64>>>>,
    ilos: Arc<Mutex<ALOS>>,
    path: Lemniscate,
    theta_0: f64,
    dt: f64,
) {
    let publisher = session.declare_publisher(topic_name).await.unwrap();
    let pub_desired_pos_theta = session
        .declare_publisher("blueboat/desired_pos")
        .await
        .unwrap();
    let mut theta_prev = theta_0;

    let mut timer = tokio::time::interval(tokio::time::Duration::from_secs_f64(dt));
    loop {
        timer.tick().await;

        let pos = {
            let pos_guard = arc_pos.lock().unwrap();
            if (*pos_guard).is_none() {
                continue;
            }
            pos_guard.unwrap()
        };
        let theta = path.comp_theta_bgd(&pos, theta_prev);
        let pos_desired = path.comp_pos(theta);
        let tau_desired = path.comp_tangent(theta);

        let (yaw, yaw_rate) = {
            let mut ilos = ilos.lock().unwrap();
            ilos.update(&pos, &pos_desired, &tau_desired, dt);
            ilos.get_references()
        };
        publish_ilos_message(&publisher, yaw, yaw_rate).await;
        publish_desired_pos_theta(&pub_desired_pos_theta, &pos_desired, theta).await;

        theta_prev = theta;
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
    if let Err(e) = publisher.put(encoded).await {
        println!("Error writing {}: {}", publisher.key_expr().as_str(), e);
    }
}

pub async fn publish_spatial_los_message(
    publisher: &Publisher<'_>,
    reduced_orientation: &Unit<Vector3<f64>>,
) {
    let t_now = std::time::SystemTime::now();
    let since_epoch = t_now.duration_since(std::time::UNIX_EPOCH).unwrap();

    let header = Header {
        stamp: Time {
            sec: since_epoch.as_secs() as i32,
            nanosec: since_epoch.subsec_nanos(),
        },
        frame_id: "".to_string(),
    };

    let los_msg = SpatialLOSMessage {
        header,
        reduced_orientation: ROSVector3 {
            x: reduced_orientation[0],
            y: reduced_orientation[1],
            z: reduced_orientation[2],
        },
    };

    let encoded = cdr::serialize::<_, _, CdrLe>(&los_msg, Infinite).unwrap();
    if let Err(e) = publisher.put(encoded).await {
        println!("Error writing {}: {}", publisher.key_expr().as_str(), e);
    }
}

pub async fn publish_desired_pos_theta(publisher: &Publisher<'_>, pos: &Vector2<f64>, theta: f64) {
    let t_now = std::time::SystemTime::now();
    let since_epoch = t_now.duration_since(std::time::UNIX_EPOCH).unwrap();

    // let header = Header {
    //     stamp: Time {
    //         sec: since_epoch.as_secs() as i32,
    //         nanosec: since_epoch.subsec_nanos(),
    //     },
    //     frame_id: "".to_string(),
    // };

    let msg = ROSVector3 {
        x: pos[0],
        y: pos[1],
        z: theta,
    };

    let encoded = cdr::serialize::<_, _, CdrLe>(&msg, Infinite).unwrap();
    if let Err(e) = publisher.put(encoded).await {
        println!("Error writing {}: {}", publisher.key_expr().as_str(), e);
    }
}

pub async fn publish_desired_spatial_pos(publisher: &Publisher<'_>, pos: &Vector3<f64>) {
    let t_now = std::time::SystemTime::now();
    let since_epoch = t_now.duration_since(std::time::UNIX_EPOCH).unwrap();

    // let header = Header {
    //     stamp: Time {
    //         sec: since_epoch.as_secs() as i32,
    //         nanosec: since_epoch.subsec_nanos(),
    //     },
    //     frame_id: "".to_string(),
    // };

    let msg = ROSVector3 {
        x: pos[0],
        y: pos[1],
        z: pos[2],
    };

    let encoded = cdr::serialize::<_, _, CdrLe>(&msg, Infinite).unwrap();
    if let Err(e) = publisher.put(encoded).await {
        println!("Error writing {}: {}", publisher.key_expr().as_str(), e);
    }
}

pub async fn position_subscriber(
    session: Session,
    topic_name: String,
    arc_pos: Arc<Mutex<Option<Vector2<f64>>>>,
) {
    let subscriber = session.declare_subscriber(topic_name).await.unwrap();

    while let Ok(sample) = subscriber.recv_async().await {
        match cdr::deserialize_from::<_, Odometry, _>(
            sample.payload().reader(),
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

pub async fn spatial_position_subscriber(
    session: Session,
    topic_name: String,
    arc_pos: Arc<Mutex<Option<Vector3<f64>>>>,
) {
    let subscriber = session.declare_subscriber(topic_name).await.unwrap();

    while let Ok(sample) = subscriber.recv_async().await {
        match cdr::deserialize_from::<_, Odometry, _>(
            sample.payload().reader(),
            cdr::size::Infinite,
        ) {
            Ok(odom) => {
                let pos = Vector3::new(
                    odom.pose.pose.position.x,
                    odom.pose.pose.position.y,
                    odom.pose.pose.position.z,
                );
                let mut pos_ref = arc_pos.lock().unwrap();
                *pos_ref = Some(pos);
            }
            Err(e) => println!("Error decoding Odometry msg: {}", e),
        }
    }
}

pub async fn update_ilos_parameters(session: Session, key_expr: String, ilos: Arc<Mutex<ILOS>>) {
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
    let subscriber = session.declare_subscriber(&key_expr).await.unwrap();

    println!("Declaring Parameter Queryable on '{key_expr}'...");
    let queryable = session
        .declare_queryable(&key_expr)
        // .complete(complete)
        .await
        .unwrap();

    loop {
        select!(
            sample = subscriber.recv_async() => {
                let sample = sample.unwrap();
                // let data: Vec<u8> = sample.payload().deserialize().unwrap();
                // match serde_json::from_str(String::from_utf8(data).unwrap().as_str()) {
                match serde_json::from_slice(&sample.payload().to_bytes()) {
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
                match query.payload() {
                    None => println!(">> [Queryable ] Received Query '{}'", query.selector()),
                    Some(query_payload) => {
                        // Refer to z_bytes.rs to see how to deserialize different types of message
                        let deserialized_payload = query_payload
                        .try_to_string()
                        .unwrap_or_else(|e| e.to_string().into());
                        println!(
                            ">> [Queryable ] Received Query '{}' with payload '{}'",
                            query.selector(),
                            deserialized_payload
                        )
                    }
                }
                let payload = serde_json::to_string(&ilos_params).unwrap().into_bytes();
                query
                    .reply(key_expr.clone(), payload.clone())
                    .encoding(Encoding::APPLICATION_JSON)
                    .await
                    .unwrap_or_else(|e| println!(">> [Queryable ] Error sending reply: {e}"));
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

#[derive(Serialize, Deserialize, PartialEq)]
struct SpatialLOSMessage {
    header: Header,
    reduced_orientation: ROSVector3,
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
