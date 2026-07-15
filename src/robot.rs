use std::f64::consts::PI;

use crate::errors::GeometryError;

/*
i wanted to be able to set hard pyhsical limits (machine capability, certification limits etc), and have user be able to set their own
self imposed ones within those constraints
*/
pub const SHOULDER_HARD_MIN_DEG: f64 = -90.0;
pub const SHOULDER_HARD_MAX_DEG: f64 = 90.0;

pub const ELBOW_HARD_MIN_DEG: f64 = 0.0;
pub const ELBOW_HARD_MAX_DEG: f64 = 150.0;

pub const EPSILON: f64 = 1e-9;

//current limits trying to be set
pub struct RequestedLimits {
    pub shoulder_min_deg: f64,
    pub shoulder_max_deg: f64,
    pub elbow_min_deg: f64,
    pub elbow_max_deg: f64,
}
//limits for a single joint
pub struct JointLimit {
    min_rad: f64,
    max_rad: f64,
}
//struct for the limit range for each joint
pub struct JointLimits {
    pub elbow: JointLimit,
    pub shoulder: JointLimit,
}
//current angles of the joints
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct JointAngles {
    pub shoulder_rad: f64,
    pub elbow_rad: f64,
}

//current state and specs of the arm
pub struct TwoLinkArm {
    link1_length: f64,
    link2_length: f64,
    limits: JointLimits,
    pub current_angles: JointAngles,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Point2 {
    pub x: f64,
    pub y: f64,
}
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ArmPose {
    elbow_pos: Point2,
    end_effector: Point2,
}

impl JointLimits {
    pub fn from_requested(requested: RequestedLimits) -> Result<Self, GeometryError> {
        Self::validate_finite(&requested)?;
        if requested.shoulder_min_deg < SHOULDER_HARD_MIN_DEG {
            return Err(GeometryError::MIN_ANGLE_NOT_MET);
        }
        if requested.elbow_min_deg < ELBOW_HARD_MIN_DEG {
            return Err(GeometryError::MIN_ANGLE_NOT_MET);
        }
        if requested.shoulder_max_deg > SHOULDER_HARD_MAX_DEG {
            return Err(GeometryError::MAX_ANGLE_EXCEEDED);
        }
        if requested.elbow_max_deg > ELBOW_HARD_MAX_DEG {
            return Err(GeometryError::MAX_ANGLE_EXCEEDED);
        }
        if requested.shoulder_min_deg > requested.shoulder_max_deg {
            return Err(GeometryError::MIN_ANGLE_NOT_POSSIBLE);
        }
        if requested.elbow_min_deg > requested.elbow_max_deg {
            return Err(GeometryError::MIN_ANGLE_NOT_POSSIBLE);
        }

        let shoulder = JointLimit {
            min_rad: requested.shoulder_min_deg.to_radians(),
            max_rad: requested.shoulder_max_deg.to_radians(),
        };

        let elbow = JointLimit {
            min_rad: requested.elbow_min_deg.to_radians(),
            max_rad: requested.elbow_max_deg.to_radians(),
        };

        Ok(JointLimits { elbow, shoulder })
    }

    pub fn validate_finite(requested: &RequestedLimits) -> Result<(), GeometryError> {
        if !requested.shoulder_max_deg.is_finite()
            || !requested.shoulder_min_deg.is_finite()
            || !requested.elbow_max_deg.is_finite()
            || !requested.elbow_min_deg.is_finite()
        {
            return Err(GeometryError::NOT_FINITE_ANGLE);
        }

        Ok(())
    }
}
impl JointLimit {
    fn contains(&self, angle_rad: f64) -> bool {
        angle_rad.is_finite() && angle_rad >= self.min_rad && angle_rad <= self.max_rad
    }
}
impl TwoLinkArm {
    pub fn set_angles(&mut self, proposed: JointAngles) -> Result<(), GeometryError> {
        let shoulder_valid = self.limits.shoulder.contains(proposed.shoulder_rad);

        let elbow_valid = self.limits.elbow.contains(proposed.elbow_rad);

        if !shoulder_valid {
            return Err(GeometryError::SHOULDER_ANGLE_OUT_OF_BOUNDS);
        }

        if !elbow_valid {
            return Err(GeometryError::ELBOW_ANGLE_OUT_OF_BOUNDS);
        }

        self.current_angles = proposed;

        Ok(())
    }
    pub fn new(
        link1_length: f64,
        link2_length: f64,
        limits: JointLimits,
        current_angles: JointAngles,
    ) -> Result<Self, GeometryError> {
        if !link1_length.is_finite() || !link2_length.is_finite() {
            return Err(GeometryError::ARM_LENGTH_NON_FINITE);
        }

        if link1_length <= 0.0 || link2_length <= 0.0 {
            return Err(GeometryError::ARM_LENGTH_OUT_OF_BOUNDS);
        }

        if !limits.shoulder.contains(current_angles.shoulder_rad) {
            return Err(GeometryError::CURRENT_ANGLES_OUT_OF_BOUNDS);
        }
        if !limits.elbow.contains(current_angles.elbow_rad) {
            return Err(GeometryError::CURRENT_ANGLES_OUT_OF_BOUNDS);
        }

        Ok(Self {
            link1_length,
            link2_length,
            limits,
            current_angles,
        })
    }

    pub fn forward_kinematics(&self) -> ArmPose {
        let cur_shoulder = self.current_angles.shoulder_rad;
        let cur_elbow = self.current_angles.elbow_rad;

        let elbow_x = self.link1_length * cur_shoulder.cos();
        let elbow_y = self.link1_length * cur_shoulder.sin();

        let forearm_world_angle = cur_elbow + cur_shoulder;

        let forearm_dx = self.link2_length * forearm_world_angle.cos();
        let forearm_dy = self.link2_length * forearm_world_angle.sin();

        let end_x = elbow_x + forearm_dx;
        let end_y = elbow_y + forearm_dy;

        let elbow_pos = Point2 {
            x: elbow_x,
            y: elbow_y,
        };
        let end_effector = Point2 { x: end_x, y: end_y };

        ArmPose {
            elbow_pos,
            end_effector,
        }
    }

    pub fn is_reachable(&self, target: Point2) -> Result<bool, GeometryError> {
        if !target.x.is_finite() || !target.y.is_finite() {
            return Err(GeometryError::POINT_NOT_FINITE);
        }

        let distance = target.x.hypot(target.y);

        let r_min = (self.link1_length - self.link2_length).abs();
        let r_max = self.link1_length + self.link2_length;

        let reachable = distance >= r_min - EPSILON && distance <= r_max + EPSILON;

        Ok(reachable)
    }

    pub fn inverse_kinematics(&self, target: Point2) -> Result<Vec<JointAngles>, GeometryError> {
        if !self.is_reachable(target)? {
            return Err(GeometryError::POINT_OUT_OF_REACH);
        }

        //find the distance to the target
        let distance = target.x.hypot(target.y);

        if distance <= EPSILON {
            return Err(GeometryError::POINT_AT_BASE);
        }

        //find the angle of the straight line to the target relative to the base
        let target_direction = target.y.atan2(target.x);

        //find the cosine of the angle between the upper arm and the straight line to target and convert it using arccos to find the angle itself
        let alpha_cos = ((self.link1_length.powi(2)) + (distance.powi(2))
            - (self.link2_length.powi(2)))
            / (2.0 * self.link1_length * distance);
        let alpha = alpha_cos.clamp(-1.0, 1.0).acos();

        //find the elbow’s relative joint angle using cosine law since we now know all the sides of the triangle
        let beta_cos = ((self.link1_length.powi(2)) + (self.link2_length.powi(2))
            - (distance.powi(2)))
            / (2.0 * self.link1_length * self.link2_length);
        let beta = beta_cos.clamp(-1.0, 1.0).acos();

        //first solution (elbow down according to how i named the signs)
        let solution_1_shoulder_angle = target_direction - alpha;
        let solution_1_elbow_angle = PI - beta;

        let solution_2_shoulder_angle = target_direction + alpha;
        let solution_2_elbow_angle = beta - PI;

        let solution_1 = JointAngles {
            shoulder_rad: solution_1_shoulder_angle,
            elbow_rad: solution_1_elbow_angle,
        };

        let solution_2 = JointAngles {
            shoulder_rad: solution_2_shoulder_angle,
            elbow_rad: solution_2_elbow_angle,
        };

        let mut solutions: Vec<JointAngles> = Vec::with_capacity(2);

        // Create an array containing the two calculated IK candidates.
        //
        // The loop runs twice:
        // first with candidate = solution_1
        // then with candidate = solution_2
        for candidate in [solution_1, solution_2] {
            // Check whether this candidate's shoulder angle is inside
            // the configured shoulder limits.
            let shoulder_valid = self.limits.shoulder.contains(candidate.shoulder_rad);

            // Check whether this candidate's elbow angle is inside
            // the configured elbow limits.
            let elbow_valid = self.limits.elbow.contains(candidate.elbow_rad);

            // Keep examining the candidate only if both joints are valid.
            if shoulder_valid && elbow_valid {
                // Look through the solutions that have already been accepted.
                //
                // `.iter()` borrows each existing JointAngles value.
                // `.any(...)` returns true if at least one existing solution
                // approximately matches the current candidate.
                let already_present = solutions.iter().any(|existing| {
                    // Compare shoulder angles using an epsilon because
                    // direct floating-point equality is unreliable.
                    let same_shoulder =
                        (existing.shoulder_rad - candidate.shoulder_rad).abs() < EPSILON;

                    // Compare elbow angles in the same way.
                    let same_elbow = (existing.elbow_rad - candidate.elbow_rad).abs() < EPSILON;

                    // The complete solution is considered equal only if
                    // both joint angles approximately match.
                    same_shoulder && same_elbow
                });

                // Add the candidate only when an equivalent solution has
                // not already been added.
                if !already_present {
                    solutions.push(candidate);
                }
            }
        }

        Ok(solutions)
    }

    pub fn select_nearest_solution(&self, solutions: &[JointAngles]) -> Option<JointAngles> {
        if solutions.is_empty() {
            return None;
        }

        let mut best_solution: Option<JointAngles> = None;

        let mut best_cost = f64::INFINITY;

        for candidate in solutions {
            let shoulder_delta = candidate.shoulder_rad - self.current_angles.shoulder_rad;
            let elbow_delta = candidate.elbow_rad - self.current_angles.elbow_rad;
            let cost = shoulder_delta.powi(2) + elbow_delta.powi(2);

            if cost < best_cost {
                best_cost = cost;
                best_solution = Some(*candidate);
            }
        }

        best_solution
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn approx_eq(actual: f64, expected: f64) -> bool {
        (actual - expected).abs() < EPSILON
    }

    #[test]
    fn forward_kinematics_straight_right() {
        let requested = RequestedLimits {
            shoulder_min_deg: SHOULDER_HARD_MIN_DEG,
            shoulder_max_deg: SHOULDER_HARD_MAX_DEG,
            elbow_min_deg: ELBOW_HARD_MIN_DEG,
            elbow_max_deg: ELBOW_HARD_MAX_DEG,
        };

        let limits = JointLimits::from_requested(requested).unwrap();

        let initial_angles = JointAngles {
            shoulder_rad: 0.0,
            elbow_rad: 0.0,
        };

        let arm = TwoLinkArm::new(1.0, 1.0, limits, initial_angles).unwrap();

        let pose = arm.forward_kinematics();

        assert!(approx_eq(pose.elbow_pos.x, 1.0));
        assert!(approx_eq(pose.elbow_pos.y, 0.0));

        assert!(approx_eq(pose.end_effector.x, 2.0));
        assert!(approx_eq(pose.end_effector.y, 0.0));
    }

    #[test]
    fn inverse_kinematics_reaches_target() {
        let requested = RequestedLimits {
            shoulder_min_deg: SHOULDER_HARD_MIN_DEG,
            shoulder_max_deg: SHOULDER_HARD_MAX_DEG,
            elbow_min_deg: ELBOW_HARD_MIN_DEG,
            elbow_max_deg: ELBOW_HARD_MAX_DEG,
        };

        let limits = JointLimits::from_requested(requested).unwrap();

        let initial_angles = JointAngles {
            shoulder_rad: 0.0,
            elbow_rad: 0.0,
        };

        let mut arm = TwoLinkArm::new(1.0, 1.0, limits, initial_angles).unwrap();

        let target = Point2 { x: 1.0, y: 1.0 };

        let solutions = arm.inverse_kinematics(target).unwrap();

        assert_eq!(solutions.len(), 1);

        let selected_solution = solutions[0];

        assert!(approx_eq(selected_solution.shoulder_rad, 0.0));
        assert!(approx_eq(selected_solution.elbow_rad, PI / 2.0));

        arm.set_angles(selected_solution).unwrap();

        let pose = arm.forward_kinematics();

        assert!(approx_eq(pose.end_effector.x, target.x));
        assert!(approx_eq(pose.end_effector.y, target.y));
    }
}
