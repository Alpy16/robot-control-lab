use crate::{
    control::{JointAccelerations, JointVelocities},
    errors::{ControlError, GeometryError},
    robot::{JointAngles, TwoLinkArm},
};

pub struct SimulatedArm {
    pub arm: TwoLinkArm,
    pub velocity: JointVelocities,
}

impl SimulatedArm {
    /// Creates a new physics simulation wrapper around an existing TwoLinkArm.
    pub fn new(arm: TwoLinkArm) -> Self {
        Self {
            arm,
            velocity: JointVelocities {
                shoulder_angular_velocity: 0.0,
                elbow_angular_velocity: 0.0,
            },
        }
    }

    pub fn step(&mut self, accelerations: JointAccelerations, dt: f64) -> Result<(), ControlError> {
        if !dt.is_finite() || dt <= 0.0 {
            return Err(ControlError::NonFiniteParameter);
        }

        //  v_next = v_current + a * dt
        let next_shoulder_vel = self.velocity.shoulder_angular_velocity
            + accelerations.shoulder_angular_acceleration * dt;
        let next_elbow_vel =
            self.velocity.elbow_angular_velocity + accelerations.elbow_angular_acceleration * dt;

        if !next_shoulder_vel.is_finite() || !next_elbow_vel.is_finite() {
            return Err(ControlError::NonFiniteState);
        }

        // position_next = position_current + v_next * dt

        let next_shoulder_rad = self.arm.current_angles.shoulder_rad + next_shoulder_vel * dt;
        let next_elbow_rad = self.arm.current_angles.elbow_rad + next_elbow_vel * dt;

        let proposed_angles = JointAngles {
            shoulder_rad: next_shoulder_rad,
            elbow_rad: next_elbow_rad,
        };

        self.arm.set_angles(proposed_angles);

        self.velocity.shoulder_angular_velocity = next_shoulder_vel;
        self.velocity.elbow_angular_velocity = next_elbow_vel;

        Ok(())
    }
}
