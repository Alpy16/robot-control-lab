mod control;
mod coordinator;
mod errors;
mod planner;
mod robot;
mod simulation;

use control::PDController;
use coordinator::move_to_target;
use robot::{JointAngles, JointLimits, Point2, RequestedLimits, TwoLinkArm};
use simulation::SimulatedArm;

fn main() {
    let requested_limits = RequestedLimits {
        shoulder_min_deg: -90.0,
        shoulder_max_deg: 90.0,
        elbow_min_deg: 0.0,
        elbow_max_deg: 150.0,
    };

    let limits = JointLimits::from_requested(requested_limits).expect("invalid joint limits");

    let initial_angles = JointAngles {
        shoulder_rad: 0.0,
        elbow_rad: 0.0,
    };

    let arm = TwoLinkArm::new(1.0, 1.0, limits, initial_angles).expect("failed to construct arm");

    let controller = PDController::new(
        8.0,  // shoulder derivative gain
        10.0, // shoulder maximum acceleration
        20.0, // shoulder proportional gain
        8.0,  // elbow derivative gain
        10.0, // elbow maximum acceleration
        20.0, // elbow proportional gain
    )
    .expect("invalid controller configuration");

    let mut simulated_arm = SimulatedArm::new(arm);

    let target = Point2 { x: 1.0, y: 1.0 };

    move_to_target(
        &mut simulated_arm,
        &controller,
        target,
        0.01,  // 100 Hz
        0.001, // angle tolerance
        0.001, // velocity tolerance
        2_000, // maximum steps
    )
    .expect("motion failed");

    println!("Final angles: {:?}", simulated_arm.arm.current_angles);

    println!("Final velocities: {:?}", simulated_arm.velocity);

    println!("Final pose: {:?}", simulated_arm.arm.forward_kinematics());
}
