use crate::{errors::GeometryError, robot::JointAngles, robot::Point2, robot::TwoLinkArm};

pub fn target_planning(arm: &TwoLinkArm, target: Point2) -> Result<JointAngles, GeometryError> {
    let target_ik = arm.inverse_kinematics(target)?;
    let solution = arm.select_nearest_solution(&target_ik);
    match solution {
        Some(solution) => Ok(solution),
        None => Err(GeometryError::NO_VIABLE_SOLUTION),
    }
}
