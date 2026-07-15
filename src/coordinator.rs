use crate::{
    control::PDController,
    errors::{ControlError, GeometryError},
    planner::target_planning,
    robot::Point2,
    simulation::SimulatedArm,
};

#[derive(Debug)]
pub enum MotionError {
    Geometry(GeometryError),
    Control(ControlError),
    Timeout,
}

pub fn move_to_target(
    sim_arm: &mut SimulatedArm,
    controller: &PDController,
    target: Point2,
    dt: f64,
    angle_tolerance: f64,
    velocity_tolerance: f64,
    max_steps: usize,
) -> Result<(), MotionError> {
    // Plan once because the Cartesian target is stationary.
    let target_angles = target_planning(&sim_arm.arm, target).map_err(MotionError::Geometry)?;
    for _ in 0..max_steps {
        let current_angles = sim_arm.arm.current_angles;
        let current_velocities = sim_arm.velocity;

        let shoulder_error = target_angles.shoulder_rad - current_angles.shoulder_rad;

        let elbow_error = target_angles.elbow_rad - current_angles.elbow_rad;

        let angles_settled =
            shoulder_error.abs() < angle_tolerance && elbow_error.abs() < angle_tolerance;

        let velocities_settled = current_velocities.shoulder_angular_velocity.abs()
            < velocity_tolerance
            && current_velocities.elbow_angular_velocity.abs() < velocity_tolerance;

        if angles_settled && velocities_settled {
            return Ok(());
        }

        let accelerations = controller
            .compute(target_angles, current_angles, current_velocities)
            .map_err(MotionError::Control)?;

        sim_arm
            .step(accelerations, dt)
            .map_err(MotionError::Control)?
    }

    Err(MotionError::Timeout)
}
