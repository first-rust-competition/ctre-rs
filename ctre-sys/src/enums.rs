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
    BufferFailure = -10,
    FirwmwareNonFRC = -11,

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
    WrongRemoteLimitSwitchSource = -504,

    // Higher Level
    IncompatibleMode = -600,
    /// Handle does not match stored map of handles
    InvalidHandle = -601,

    // Firmware Versions
    FeatureRequiresHigherFirm = -700,
    MotorControllerFeatureRequiresHigherFirm = -701,
    ConfigFactoryDefaultRequiresHigherFirm = -702,

    // Operating system centric
    LibraryCouldNotBeLoaded = -800,
    MissingRoutineInLibrary = -801,
    ResourceNotAvailable = -802,

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

#[repr(transparent)]
/// Signal enumeration for generic signal access.
#[derive(Debug, Copy, Clone, PartialEq, Eq, Hash)]
pub struct ParamEnum(pub u32);
impl ParamEnum {
    pub const OnBoot_BrakeMode: ParamEnum = ParamEnum(31);
    pub const QuadFilterEn: ParamEnum = ParamEnum(91);
    pub const QuadIdxPolarity: ParamEnum = ParamEnum(108);
    pub const MotionProfileHasUnderrunErr: ParamEnum = ParamEnum(119);
    /// motionProfileTrajectoryPeriod
    pub const MotionProfileTrajectoryPointDurationMs: ParamEnum = ParamEnum(120);
    pub const MotionProfileTrajectoryInterpolDis: ParamEnum = ParamEnum(121);

    pub const StatusFramePeriod: ParamEnum = ParamEnum(300);
    pub const OpenloopRamp: ParamEnum = ParamEnum(301);
    pub const ClosedloopRamp: ParamEnum = ParamEnum(302);
    pub const NeutralDeadband: ParamEnum = ParamEnum(303);

    pub const PeakPosOutput: ParamEnum = ParamEnum(305);
    pub const NominalPosOutput: ParamEnum = ParamEnum(306);
    pub const PeakNegOutput: ParamEnum = ParamEnum(307);
    pub const NominalNegOutput: ParamEnum = ParamEnum(308);

    pub const ProfileParamSlot_P: ParamEnum = ParamEnum(310);
    pub const ProfileParamSlot_I: ParamEnum = ParamEnum(311);
    pub const ProfileParamSlot_D: ParamEnum = ParamEnum(312);
    pub const ProfileParamSlot_F: ParamEnum = ParamEnum(313);
    pub const ProfileParamSlot_IZone: ParamEnum = ParamEnum(314);
    pub const ProfileParamSlot_AllowableErr: ParamEnum = ParamEnum(315);
    pub const ProfileParamSlot_MaxIAccum: ParamEnum = ParamEnum(316);
    pub const ProfileParamSlot_PeakOutput: ParamEnum = ParamEnum(317);

    pub const ClearPositionOnLimitF: ParamEnum = ParamEnum(320);
    pub const ClearPositionOnLimitR: ParamEnum = ParamEnum(321);
    pub const ClearPositionOnQuadIdx: ParamEnum = ParamEnum(322);

    #[deprecated(note = "use `ParamEnum::ClearPositionOnQuadIdx` instead")]
    pub const ClearPositionOnIdx: ParamEnum = ParamEnum::ClearPositionOnQuadIdx;
    #[deprecated(note = "use `ParamEnum::ClearPositionOnLimitF` instead")]
    pub const ClearPosOnLimitF: ParamEnum = ParamEnum::ClearPositionOnLimitF;
    #[deprecated(note = "use `ParamEnum::ClearPositionOnLimitR` instead")]
    pub const ClearPosOnLimitR: ParamEnum = ParamEnum::ClearPositionOnLimitR;

    pub const SampleVelocityPeriod: ParamEnum = ParamEnum(325);
    pub const SampleVelocityWindow: ParamEnum = ParamEnum(326);

    pub const FeedbackSensorType: ParamEnum = ParamEnum(330);
    pub const SelectedSensorPosition: ParamEnum = ParamEnum(331);
    pub const FeedbackNotContinuous: ParamEnum = ParamEnum(332);
    pub const RemoteSensorSource: ParamEnum = ParamEnum(333); // RemoteSensorSource_t
    /// [0,62] DeviceID
    pub const RemoteSensorDeviceID: ParamEnum = ParamEnum(334);
    /// feedbackDevice_t (ordinal is the register)
    pub const SensorTerm: ParamEnum = ParamEnum(335);
    pub const RemoteSensorClosedLoopDisableNeutralOnLOS: ParamEnum = ParamEnum(336);
    /// auxPIDPolarity
    pub const PIDLoopPolarity: ParamEnum = ParamEnum(337);
    pub const PIDLoopPeriod: ParamEnum = ParamEnum(338);
    pub const SelectedSensorCoefficient: ParamEnum = ParamEnum(339);

    pub const ForwardSoftLimitThreshold: ParamEnum = ParamEnum(340);
    pub const ReverseSoftLimitThreshold: ParamEnum = ParamEnum(341);
    pub const ForwardSoftLimitEnable: ParamEnum = ParamEnum(342);
    pub const ReverseSoftLimitEnable: ParamEnum = ParamEnum(343);

    /// voltageCompSaturation
    pub const NominalBatteryVoltage: ParamEnum = ParamEnum(350);
    /// voltageMeasurementFilter
    pub const BatteryVoltageFilterSize: ParamEnum = ParamEnum(351);

    pub const ContinuousCurrentLimitAmps: ParamEnum = ParamEnum(360);
    pub const PeakCurrentLimitMs: ParamEnum = ParamEnum(361);
    pub const PeakCurrentLimitAmps: ParamEnum = ParamEnum(362);

    pub const ClosedLoopIAccum: ParamEnum = ParamEnum(370);

    pub const CustomParam: ParamEnum = ParamEnum(380);

    pub const StickyFaults: ParamEnum = ParamEnum(390);

    pub const AnalogPosition: ParamEnum = ParamEnum(400);
    pub const QuadraturePosition: ParamEnum = ParamEnum(401);
    pub const PulseWidthPosition: ParamEnum = ParamEnum(402);

    pub const MotMag_Accel: ParamEnum = ParamEnum(410);
    pub const MotMag_VelCruise: ParamEnum = ParamEnum(411);
    pub const MotMag_SCurveLevel: ParamEnum = ParamEnum(412);

    /// ordinal (fwd=0,reverse=1), @see LimitSwitchSource_t
    pub const LimitSwitchSource: ParamEnum = ParamEnum(421);
    /// ordinal (fwd=0,reverse=1). @see LimitSwitchNormClosedAndDis_t
    pub const LimitSwitchNormClosedAndDis: ParamEnum = ParamEnum(422);
    pub const LimitSwitchDisableNeutralOnLOS: ParamEnum = ParamEnum(423);
    pub const LimitSwitchRemoteDevID: ParamEnum = ParamEnum(424);
    pub const SoftLimitDisableNeutralOnLOS: ParamEnum = ParamEnum(425);

    pub const PulseWidthPeriod_EdgesPerRot: ParamEnum = ParamEnum(430);
    pub const PulseWidthPeriod_FilterWindowSz: ParamEnum = ParamEnum(431);

    pub const YawOffset: ParamEnum = ParamEnum(160);
    pub const CompassOffset: ParamEnum = ParamEnum(161);
    pub const BetaGain: ParamEnum = ParamEnum(162);
    pub const EnableCompassFusion: ParamEnum = ParamEnum(163);
    pub const GyroNoMotionCal: ParamEnum = ParamEnum(164);
    pub const EnterCalibration: ParamEnum = ParamEnum(165);
    pub const FusedHeadingOffset: ParamEnum = ParamEnum(166);
    pub const StatusFrameRate: ParamEnum = ParamEnum(169);
    pub const AccumZ: ParamEnum = ParamEnum(170);
    pub const TempCompDisable: ParamEnum = ParamEnum(171);
    pub const MotionMeas_tap_threshX: ParamEnum = ParamEnum(172);
    pub const MotionMeas_tap_threshY: ParamEnum = ParamEnum(173);
    pub const MotionMeas_tap_threshZ: ParamEnum = ParamEnum(174);
    pub const MotionMeas_tap_count: ParamEnum = ParamEnum(175);
    pub const MotionMeas_tap_time: ParamEnum = ParamEnum(176);
    pub const MotionMeas_tap_time_multi: ParamEnum = ParamEnum(177);
    pub const MotionMeas_shake_reject_thresh: ParamEnum = ParamEnum(178);
    pub const MotionMeas_shake_reject_time: ParamEnum = ParamEnum(179);
    pub const MotionMeas_shake_reject_timeout: ParamEnum = ParamEnum(180);

    pub const DefaultConfig: ParamEnum = ParamEnum(500);
}
