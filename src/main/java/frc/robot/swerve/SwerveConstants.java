package frc.robot.swerve;

import com.ctre.phoenix6.configs.*;
import edu.wpi.first.math.geometry.Translation2d;

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
    public static final double STEERING_MULTIPLIER = 0.6;

    public static final Slot0Configs DRIVE_PID_GAINS =
            new Slot0Configs().withKP(0.0).withKI(0.0).withKD(0.0).withKV(0.6).withKS(0.6);
    public static final Slot0Configs ANGLE_PID_GAINS =
            new Slot0Configs()
                    .withKP(28.0)
                    .withKI(0.0)
                    .withKD(0.0);
    public static final double kF = 0.28;

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
                    .withSlot0(DRIVE_PID_GAINS)
                    .withVoltage(VOLTAGE_CONFIGS)
                    .withCurrentLimits(CURRENT_LIMITS_CONFIGS)
                    .withFeedback(FEEDBACK_CONFIGS_DRIVE);
    public static final TalonFXConfiguration ANGLE_MOTOR_CONFIGS =
            new TalonFXConfiguration()
                    .withSlot0(ANGLE_PID_GAINS)
                    .withVoltage(VOLTAGE_CONFIGS)
                    .withCurrentLimits(CURRENT_LIMITS_CONFIGS)
                    .withFeedback(FEEDBACK_CONFIGS_ANGLE)
                    .withMotorOutput(MOTOR_OUTPUT_CONFIGS);

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
