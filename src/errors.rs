#[derive(Debug, Clone, Copy, PartialEq)]

pub enum GeometryError {
    MAX_ANGLE_EXCEEDED,
    MIN_ANGLE_NOT_MET,
    MIN_ANGLE_NOT_POSSIBLE,
    NOT_FINITE_ANGLE,
    SHOULDER_ANGLE_OUT_OF_BOUNDS,
    ELBOW_ANGLE_OUT_OF_BOUNDS,
    ARM_LENGTH_NON_FINITE,
    ARM_LENGTH_OUT_OF_BOUNDS,
    CURRENT_ANGLES_OUT_OF_BOUNDS,
    POINT_OUT_OF_REACH,
    POINT_NOT_FINITE,
    POINT_AT_BASE,
    NO_VIABLE_SOLUTION,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ControlError {
    NonFiniteParameter,
    NegativeGain,
    NonPositiveAccelerationLimit,
    NonFiniteState,
}
#[derive(Debug)]
pub enum MotionError {
    Geometry(GeometryError),
    Control(ControlError),
    Timeout,
}
