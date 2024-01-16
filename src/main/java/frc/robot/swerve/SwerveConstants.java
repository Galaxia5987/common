package frc.robot.swerve;

import com.ctre.phoenix6.configs.*;
import edu.wpi.first.math.geometry.Translation2d;
import lib.webconstants.LoggedTunableNumber;

public class SwerveConstants {
    public static final double[] OFFSETS = {
            0.5665233141630829,0.26218280655457016,0.004864650121616253,0.5487426137185654
    };

    public static final double VOLT_COMP_SATURATION = 12;
    public static final CurrentLimitsConfigs CURRENT_LIMITS_CONFIGS =
            new CurrentLimitsConfigs().withSupplyCurrentLimit(50).withStatorCurrentLimit(50);
    public static final double ROBOT_WIDTH = 0.512; // [m]
    public static final double ROBOT_LENGTH = 0.67; // [m]
    public static final double WHEEL_DIAMETER = 0.099; // [m]
    public static final double DRIVE_REDUCTION = (1 / 2.0) * (24.0 / 22.0) * (15.0 / 45.0);
    public static final double ANGLE_REDUCTION = (14.0 / 72.0) * 0.5;
    public static final double DRIVE_MOTOR_MOMENT_OF_INERTIA = 0.025;
    public static final double ANGLE_MOTOR_MOMENT_OF_INERTIA = 0.004;

    public static final double NEUTRAL_DEADBAND = 0.03;
    public static final double XBOX_DEADBAND = 0.15;
    public static final LoggedTunableNumber STEERING_MULTIPLIER = new LoggedTunableNumber("Steering multiplier", 0.6);

    public static final LoggedTunableNumber DRIVE_KP = new LoggedTunableNumber("Swerve Drive/PID/driveKP",0.0);
    public static final LoggedTunableNumber DRIVE_KI = new LoggedTunableNumber("Swerve Drive/PID/driveKI",0.0);
    public static final LoggedTunableNumber DRIVE_KD = new LoggedTunableNumber("Swerve Drive/PID/driveKD",0.0);
    public static final LoggedTunableNumber DRIVE_KV = new LoggedTunableNumber("Swerve Drive/PID/driveKV",0.6);
    public static final LoggedTunableNumber DRIVE_KS = new LoggedTunableNumber("Swerve Drive/PID/driveKS",0.6);

    public static final LoggedTunableNumber ANGLE_KP = new LoggedTunableNumber("Swerve Drive/PID/angleKP",28.0);
    public static final LoggedTunableNumber ANGLE_KI = new LoggedTunableNumber("Swerve Drive/PID/angleKI",0.0);
    public static final LoggedTunableNumber ANGLE_KD = new LoggedTunableNumber("Swerve Drive/PID/angleKD",0.0);
    public static final LoggedTunableNumber ANGLE_KS = new LoggedTunableNumber("Swerve Drive/PID/angleKS",0.28);

    public static final LoggedTunableNumber[] PID_VALUES = new LoggedTunableNumber[]{DRIVE_KP, DRIVE_KI, DRIVE_KD, DRIVE_KV, DRIVE_KS, ANGLE_KP, ANGLE_KI, ANGLE_KD, ANGLE_KS};

    public static final VoltageConfigs VOLTAGE_CONFIGS =
            new VoltageConfigs()
                    .withPeakForwardVoltage(VOLT_COMP_SATURATION)
                    .withPeakReverseVoltage(VOLT_COMP_SATURATION);
    public static final FeedbackConfigs FEEDBACK_CONFIGS_DRIVE =
            new FeedbackConfigs()
                    .withRotorToSensorRatio(1)
                    .withSensorToMechanismRatio(1 / DRIVE_REDUCTION);
    public static final FeedbackConfigs FEEDBACK_CONFIGS_ANGLE =
            new FeedbackConfigs()
                    .withRotorToSensorRatio(1)
                    .withSensorToMechanismRatio(1 / ANGLE_REDUCTION);
    public static final MotorOutputConfigs MOTOR_OUTPUT_CONFIGS =
            new MotorOutputConfigs().withDutyCycleNeutralDeadband(NEUTRAL_DEADBAND);

    public static final TalonFXConfiguration DRIVE_MOTOR_CONFIGS =
            new TalonFXConfiguration()
                    .withVoltage(VOLTAGE_CONFIGS)
                    .withCurrentLimits(CURRENT_LIMITS_CONFIGS)
                    .withFeedback(FEEDBACK_CONFIGS_DRIVE);
    public static final TalonFXConfiguration ANGLE_MOTOR_CONFIGS =
            new TalonFXConfiguration()
                    .withVoltage(VOLTAGE_CONFIGS)
                    .withCurrentLimits(CURRENT_LIMITS_CONFIGS)
                    .withFeedback(FEEDBACK_CONFIGS_ANGLE)
                    .withMotorOutput(MOTOR_OUTPUT_CONFIGS);

    public static final LoggedTunableNumber SIM_DRIVE_KP = new LoggedTunableNumber("Swerve Drive/PID/sim driveKP", 3.5);
    public static final LoggedTunableNumber SIM_DRIVE_KI = new LoggedTunableNumber("Swerve Drive/PID/sim driveKI", 0);
    public static final LoggedTunableNumber SIM_DRIVE_KD = new LoggedTunableNumber("Swerve Drive/PID/sim driveKD", 0);
    public static final LoggedTunableNumber SIM_ANGLE_KP = new LoggedTunableNumber("Swerve Drive/PID/sim angleKP", 8);
    public static final LoggedTunableNumber SIM_ANGLE_KI = new LoggedTunableNumber("Swerve Drive/PID/sim angleKI", 0);
    public static final LoggedTunableNumber SIM_ANGLE_KD = new LoggedTunableNumber("Swerve Drive/PID/sim angleKD", 0);

    public static final LoggedTunableNumber[] SIM_PID_VALUES = new LoggedTunableNumber[]{SIM_DRIVE_KP,SIM_DRIVE_KI,SIM_DRIVE_KD,SIM_ANGLE_KP,SIM_ANGLE_KI,SIM_ANGLE_KD};

    public static final double MAX_X_Y_VELOCITY =
            6000.0
                    / 60.0
                    * // [m/s]
                    DRIVE_REDUCTION
                    * WHEEL_DIAMETER
                    * Math.PI;

    public static final double MAX_OMEGA_VELOCITY =
            MAX_X_Y_VELOCITY
                    / // [m/s]
                    Math.sqrt(
                            (ROBOT_LENGTH / 2) * (ROBOT_LENGTH / 2)
                                    + (ROBOT_WIDTH / 2) * (ROBOT_WIDTH / 2));

    public static final Translation2d[] WHEEL_POSITIONS = {
        new Translation2d(ROBOT_LENGTH / 2, ROBOT_WIDTH / 2), // FL
        new Translation2d(ROBOT_LENGTH / 2, -ROBOT_WIDTH / 2), // FR
        new Translation2d(-ROBOT_LENGTH / 2, ROBOT_WIDTH / 2), // RL
        new Translation2d(-ROBOT_LENGTH / 2, -ROBOT_WIDTH / 2)
    }; // RR

    public static final double[] FRONT_LEFT_MOTION_MAGIC_CONFIGS = {
        3.5, 0, 0, 0.000_65, 1, 0, 0, 10, 5, 1
    };
    public static final double[] FRONT_RIGHT_MOTION_MAGIC_CONFIGS = {
        3.5, 0, 0, 0.000_65, 1, 0, 0, 10, 5, 1
    };
    public static final double[] REAR_LEFT_MOTION_MAGIC_CONFIGS = {
        3.5, 0, 0, 0.000_65, 1, 0, 0, 10, 5, 1
    };
    public static final double[] REAR_RIGHT_MOTION_MAGIC_CONFIGS = {
        3.5, 0, 0, 0.000_65, 1, 0, 0, 10, 5, 1
    };

    public static final double[][] motionMagicConfigs = {
        FRONT_LEFT_MOTION_MAGIC_CONFIGS,
        FRONT_RIGHT_MOTION_MAGIC_CONFIGS,
        REAR_LEFT_MOTION_MAGIC_CONFIGS,
        REAR_RIGHT_MOTION_MAGIC_CONFIGS
    };

    public static final double ODOMETRY_FREQUENCY = 250;
}
