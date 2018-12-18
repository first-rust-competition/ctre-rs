//! Low-level enums common to all CTRE Phoenix devices.
#![allow(non_camel_case_types, non_upper_case_globals)]

#[repr(i32)]
#[derive(Debug, Copy, Clone, PartialEq, Eq, Hash)]
pub enum ErrorCode {
    /// No Error - Function executed as expected
    OK = 0,

    // CAN-Related
    CAN_MSG_STALE = 1,
    /// Could not transmit the CAN frame.
    TxFailed = -1,
    /// Caller passed an invalid param
    InvalidParamValue = -2,
    /// CAN frame has not been received within specified period of time.
    RxTimeout = -3,
    /// Not used.
    TxTimeout = -4,
    /// Specified CAN Id is invalid.
    UnexpectedArbId = -5,
    /// Caller attempted to insert data into a buffer that is full.
    BufferFull = 6,
    CAN_OVERFLOW = -6,
    /// Sensor is not present
    SensorNotPresent = -7,
    FirmwareTooOld = -8,
    CouldNotChangePeriod = -9,

    // General
    /// User Specified General Error
    GeneralError = -100,

    // Signal
    /// Have not received an value response for signal.
    SigNotUpdated = -200,
    NotAllPIDValuesUpdated = -201,

    // Gadgeteer Port Error Codes
    // These include errors between ports and modules
    GEN_PORT_ERROR = -300,
    PORT_MODULE_TYPE_MISMATCH = -301,

    // Gadgeteer Module Error Codes
    // These apply only to the module units themselves
    GEN_MODULE_ERROR = -400,
    MODULE_NOT_INIT_SET_ERROR = -401,
    MODULE_NOT_INIT_GET_ERROR = -402,

    // API
    WheelRadiusTooSmall = -500,
    TicksPerRevZero = -501,
    DistanceBetweenWheelsTooSmall = -502,
    GainsAreNotSet = -503,

    // Higher Level
    IncompatibleMode = -600,
    /// Handle does not match stored map of handles
    InvalidHandle = -601,

    // Firmware Versions
    FeatureRequiresHigherFirm = -700,
    MotorControllerFeatureRequiresHigherFirm = -701,
    ConfigFactoryDefaultRequiresHigherFirm = -702,

    // CAN Related
    /// Special Code for "isSensorPresent"
    PulseWidthSensorNotPresent = 10,

    // General
    GeneralWarning = 100,
    /// feature not implement in the API or firmware
    FeatureNotSupported = 101,
    /// feature not implement in the API
    NotImplemented = 102,
    FirmVersionCouldNotBeRetrieved = 103,
    /// feature will be release in an upcoming release
    FeaturesNotAvailableYet = 104,
    /// Current control mode of motor controller not valid for this call
    ControlModeNotValid = 105,

    ControlModeNotSupportedYet = 106,
    AuxiliaryPIDNotSupportedYet = 107,
    RemoteSensorsNotSupportedYet = 108,
    MotProfFirmThreshold = 109,
    MotProfFirmThreshold2 = 110,
}
impl ErrorCode {
    pub const TalonFeatureRequiresHigherFirm: ErrorCode =
        ErrorCode::MotorControllerFeatureRequiresHigherFirm;
}

#[repr(u32)]
/// Signal enumeration for generic signal access.
#[derive(Debug, Copy, Clone, PartialEq, Eq, Hash)]
pub enum ParamEnum {
    OnBoot_BrakeMode = 31,
    QuadFilterEn = 91,
    QuadIdxPolarity = 108,
    #[deprecated(note = "Use `ParamEnum::ClearPositionOnQuadIdx` instead")]
    ClearPositionOnIdx = 100,
    MotionProfileHasUnderrunErr = 119,
    MotionProfileTrajectoryPointDurationMs = 120,
    #[deprecated(note = "Use `ParamEnum::ClearPositionOnLimitF` instead")]
    ClearPosOnLimitF = 144,
    #[deprecated(note = "Use `ParamEnum::ClearPositionOnLimitR` instead")]
    ClearPosOnLimitR = 145,

    StatusFramePeriod = 300,
    OpenloopRamp = 301,
    ClosedloopRamp = 302,
    NeutralDeadband = 303,

    PeakPosOutput = 305,
    NominalPosOutput = 306,
    PeakNegOutput = 307,
    NominalNegOutput = 308,

    ProfileParamSlot_P = 310,
    ProfileParamSlot_I = 311,
    ProfileParamSlot_D = 312,
    ProfileParamSlot_F = 313,
    ProfileParamSlot_IZone = 314,
    ProfileParamSlot_AllowableErr = 315,
    ProfileParamSlot_MaxIAccum = 316,
    ProfileParamSlot_PeakOutput = 317,

    ClearPositionOnLimitF = 320,
    ClearPositionOnLimitR = 321,
    ClearPositionOnQuadIdx = 322,

    SampleVelocityPeriod = 325,
    SampleVelocityWindow = 326,

    FeedbackSensorType = 330,
    SelectedSensorPosition = 331,
    FeedbackNotContinuous = 332,
    RemoteSensorSource = 333, // RemoteSensorSource_t
    /// [0,62] DeviceID
    RemoteSensorDeviceID = 334,
    SensorTerm = 335, // feedbackDevice_t (ordinal is the register)
    RemoteSensorClosedLoopDisableNeutralOnLOS = 336,
    /// auxPIDPolarity
    PIDLoopPolarity = 337,
    PIDLoopPeriod = 338,
    SelectedSensorCoefficient = 339,

    ForwardSoftLimitThreshold = 340,
    ReverseSoftLimitThreshold = 341,
    ForwardSoftLimitEnable = 342,
    ReverseSoftLimitEnable = 343,

    /// voltageCompSaturation
    NominalBatteryVoltage = 350,
    /// voltageMeasurementFilter
    BatteryVoltageFilterSize = 351,

    ContinuousCurrentLimitAmps = 360,
    PeakCurrentLimitMs = 361,
    PeakCurrentLimitAmps = 362,

    ClosedLoopIAccum = 370,

    CustomParam = 380,

    StickyFaults = 390,

    AnalogPosition = 400,
    QuadraturePosition = 401,
    PulseWidthPosition = 402,

    MotMag_Accel = 410,
    MotMag_VelCruise = 411,

    /// ordinal (fwd=0,reverse=1), @see LimitSwitchSource_t
    LimitSwitchSource = 421,
    /// ordinal (fwd=0,reverse=1). @see LimitSwitchNormClosedAndDis_t
    LimitSwitchNormClosedAndDis = 422,
    LimitSwitchDisableNeutralOnLOS = 423,
    LimitSwitchRemoteDevID = 424,
    SoftLimitDisableNeutralOnLOS = 425,

    PulseWidthPeriod_EdgesPerRot = 430,
    PulseWidthPeriod_FilterWindowSz = 431,

    YawOffset = 160,
    CompassOffset = 161,
    BetaGain = 162,
    EnableCompassFusion = 163,
    GyroNoMotionCal = 164,
    EnterCalibration = 165,
    FusedHeadingOffset = 166,
    StatusFrameRate = 169,
    AccumZ = 170,
    TempCompDisable = 171,
    MotionMeas_tap_threshX = 172,
    MotionMeas_tap_threshY = 173,
    MotionMeas_tap_threshZ = 174,
    MotionMeas_tap_count = 175,
    MotionMeas_tap_time = 176,
    MotionMeas_tap_time_multi = 177,
    MotionMeas_shake_reject_thresh = 178,
    MotionMeas_shake_reject_time = 179,
    MotionMeas_shake_reject_timeout = 180,

    DefaultConfig = 500,
}
