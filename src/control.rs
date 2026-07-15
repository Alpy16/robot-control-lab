use crate::{errors::ControlError, robot::JointAngles};

#[derive(Debug, Clone, Copy)]
pub struct JointVelocities {
    pub(crate) shoulder_angular_velocity: f64,
    pub(crate) elbow_angular_velocity: f64,
}

#[derive(Debug, Clone, Copy)]
pub struct JointAccelerations {
    pub(crate) shoulder_angular_acceleration: f64,
    pub(crate) elbow_angular_acceleration: f64,
}

pub struct PDController {
    shoulder_proportional_gain: f64,
    shoulder_derivative_gain: f64,
    elbow_proportional_gain: f64,
    elbow_derivative_gain: f64,
    elbow_maximum_acceleration: f64,
    shoulder_maximum_acceleration: f64,
}

impl PDController {
    fn validate(&self) -> Result<(), ControlError> {
        let all_finite = self.shoulder_proportional_gain.is_finite()
            && self.shoulder_derivative_gain.is_finite()
            && self.elbow_proportional_gain.is_finite()
            && self.elbow_derivative_gain.is_finite()
            && self.shoulder_maximum_acceleration.is_finite()
            && self.elbow_maximum_acceleration.is_finite();

        if !all_finite {
            return Err(ControlError::NonFiniteParameter);
        }

        let gains_non_negative = self.shoulder_proportional_gain >= 0.0
            && self.shoulder_derivative_gain >= 0.0
            && self.elbow_proportional_gain >= 0.0
            && self.elbow_derivative_gain >= 0.0;

        if !gains_non_negative {
            return Err(ControlError::NegativeGain);
        }

        let acceleration_limits_positive =
            self.shoulder_maximum_acceleration > 0.0 && self.elbow_maximum_acceleration > 0.0;

        if !acceleration_limits_positive {
            return Err(ControlError::NonPositiveAccelerationLimit);
        }

        Ok(())
    }

    pub fn new(
        shoulder_deriv_gain: f64,
        shoulder_max_acceleration: f64,
        shoulder_prop_gain: f64,
        elbow_deriv_gain: f64,
        elbow_max_acceleration: f64,
        elbow_prop_gain: f64,
    ) -> Result<Self, ControlError> {
        let gains = Self {
            shoulder_derivative_gain: shoulder_deriv_gain,
            shoulder_maximum_acceleration: shoulder_max_acceleration,
            shoulder_proportional_gain: shoulder_prop_gain,
            elbow_derivative_gain: elbow_deriv_gain,
            elbow_maximum_acceleration: elbow_max_acceleration,
            elbow_proportional_gain: elbow_prop_gain,
        };

        gains.validate()?;

        Ok(gains)
    }

    pub fn compute(
        &self,
        target: JointAngles,
        current_angle: JointAngles,
        current_vel: JointVelocities,
    ) -> Result<JointAccelerations, ControlError> {
        // Destructuring makes it incredibly easy to track them separately
        let JointAngles {
            shoulder_rad: target_shoulder,
            elbow_rad: target_elbow,
        } = target;
        let JointAngles {
            shoulder_rad: curr_shoulder,
            elbow_rad: curr_elbow,
        } = current_angle;
        let JointVelocities {
            shoulder_angular_velocity: current_shoulder_vel,
            elbow_angular_velocity: current_elbow_vel,
        } = current_vel;

        let all_finite = curr_elbow.is_finite()
            && curr_shoulder.is_finite()
            && target_elbow.is_finite()
            && target_shoulder.is_finite()
            && current_elbow_vel.is_finite()
            && current_shoulder_vel.is_finite();

        if !all_finite {
            return Err(ControlError::NonFiniteState);
        }
        // Now you have independent f64 variables to calculate your errors
        let shoulder_error = target_shoulder - curr_shoulder;
        let elbow_error = target_elbow - curr_elbow;

        let raw_shoulder_acc = self.shoulder_proportional_gain * shoulder_error
            - self.shoulder_derivative_gain * current_shoulder_vel;

        let raw_elbow_acc = self.elbow_proportional_gain * elbow_error
            - self.elbow_derivative_gain * current_elbow_vel;

        let shoulder_acceleration = raw_shoulder_acc.clamp(
            -self.shoulder_maximum_acceleration,
            self.shoulder_maximum_acceleration,
        );

        let elbow_acceleration = raw_elbow_acc.clamp(
            -self.elbow_maximum_acceleration,
            self.elbow_maximum_acceleration,
        );

        Ok(JointAccelerations {
            shoulder_angular_acceleration: (shoulder_acceleration),
            elbow_angular_acceleration: (elbow_acceleration),
        })
    }

    pub fn brake(&self, current_vel: JointVelocities) -> Result<JointAccelerations, ControlError> {
        let JointVelocities {
            shoulder_angular_velocity: current_shoulder_vel,
            elbow_angular_velocity: current_elbow_vel,
        } = current_vel;

        let all_finite = current_shoulder_vel.is_finite() && current_elbow_vel.is_finite();
        if !all_finite {
            return Err(ControlError::NonFiniteState);
        }

        let raw_shoulder_acc = -self.shoulder_derivative_gain * current_shoulder_vel;
        let raw_elbow_acc = -self.elbow_derivative_gain * current_elbow_vel;

        let shoulder_acceleration = raw_shoulder_acc.clamp(
            -self.shoulder_maximum_acceleration,
            self.shoulder_maximum_acceleration,
        );

        let elbow_acceleration = raw_elbow_acc.clamp(
            -self.elbow_maximum_acceleration,
            self.elbow_maximum_acceleration,
        );

        Ok(JointAccelerations {
            shoulder_angular_acceleration: shoulder_acceleration,
            elbow_angular_acceleration: elbow_acceleration,
        })
    }
}
