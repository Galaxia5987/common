package frc.robot.swerve;

import com.ctre.phoenix6.configs.*;
import edu.wpi.first.math.geometry.Translation2d;
import lib.webconstants.LoggedTunableNumber;

public class SwerveConstants {
    public static final double NEO_CURRENT_LIMIT = 40;
    public static final double NEO_550_CURRENT_LIMIT = 20;
    public static final CurrentLimitsConfigs TALON_FX_CURRENT_LIMIT_CONFIGS =
            new CurrentLimitsConfigs().withSupplyCurrentLimit(50).withStatorCurrentLimit(50);

    public static final double VOLT_COMP_SATURATION = 12;
    public static final double NEUTRAL_DEADBAND = 0.03;
    public static final double XBOX_DEADBAND = 0.15;
    public static final LoggedTunableNumber STEERING_MULTIPLIER =
            new LoggedTunableNumber("Steering multiplier", 0.6);
    public static final LoggedTunableNumber DRIVE_KP =
            new LoggedTunableNumber("Swerve Drive/PID/driveKP");
    public static final LoggedTunableNumber DRIVE_KI =
            new LoggedTunableNumber("Swerve Drive/PID/driveKI");
    public static final LoggedTunableNumber DRIVE_KD =
            new LoggedTunableNumber("Swerve Drive/PID/driveKD");
    public static final LoggedTunableNumber DRIVE_KV =
            new LoggedTunableNumber("Swerve Drive/PID/driveKV");
    public static final LoggedTunableNumber DRIVE_KS =
            new LoggedTunableNumber("Swerve Drive/PID/driveKS");
    public static final LoggedTunableNumber DRIVE_KA =
            new LoggedTunableNumber("Swerve Drive/PID/driveKA");
    public static final LoggedTunableNumber ANGLE_KP =
            new LoggedTunableNumber("Swerve Drive/PID/angleKP");
    public static final LoggedTunableNumber ANGLE_KI =
            new LoggedTunableNumber("Swerve Drive/PID/angleKI");
    public static final LoggedTunableNumber ANGLE_KD =
            new LoggedTunableNumber("Swerve Drive/PID/angleKD");
    public static final LoggedTunableNumber ANGLE_KS =
            new LoggedTunableNumber("Swerve Drive/PID/angleKS");
    public static final LoggedTunableNumber[] PID_VALUES =
            new LoggedTunableNumber[] {
                DRIVE_KP, DRIVE_KI, DRIVE_KD, DRIVE_KV, DRIVE_KS, DRIVE_KA, ANGLE_KP, ANGLE_KI,
                ANGLE_KD, ANGLE_KS
            };
    public static final LoggedTunableNumber ROTATION_KP =
            new LoggedTunableNumber("Swerve Drive/Rotation/rotationKP");
    public static final LoggedTunableNumber ROTATION_KI =
            new LoggedTunableNumber("Swerve Drive/Rotation/rotationKI");
    public static final LoggedTunableNumber ROTATION_KD =
            new LoggedTunableNumber("Swerve Drive/Rotation/rotationKD");
    public static final LoggedTunableNumber ROTATION_KDIETER =
            new LoggedTunableNumber("Swerve Drive/Rotation/rotationKDIETER");

    public static final VoltageConfigs VOLTAGE_CONFIGS =
            new VoltageConfigs()
                    .withPeakForwardVoltage(VOLT_COMP_SATURATION)
                    .withPeakReverseVoltage(VOLT_COMP_SATURATION);
    public static final MotorOutputConfigs MOTOR_OUTPUT_CONFIGS =
            new MotorOutputConfigs().withDutyCycleNeutralDeadband(NEUTRAL_DEADBAND);
    public static final double ODOMETRY_FREQUENCY = 250;
    public static double ROBOT_WIDTH;
    public static double ROBOT_LENGTH;
    public static Translation2d[] WHEEL_POSITIONS;
    public static double WHEEL_DIAMETER;
    public static double DRIVE_REDUCTION;
    public static final FeedbackConfigs FEEDBACK_CONFIGS_DRIVE =
            new FeedbackConfigs()
                    .withRotorToSensorRatio(1)
                    .withSensorToMechanismRatio(1 / DRIVE_REDUCTION);
    public static final TalonFXConfiguration DRIVE_MOTOR_CONFIGS =
            new TalonFXConfiguration()
                    .withVoltage(VOLTAGE_CONFIGS)
                    .withCurrentLimits(TALON_FX_CURRENT_LIMIT_CONFIGS)
                    .withFeedback(FEEDBACK_CONFIGS_DRIVE);
    public static double ANGLE_REDUCTION;
    public static final FeedbackConfigs FEEDBACK_CONFIGS_ANGLE =
            new FeedbackConfigs()
                    .withRotorToSensorRatio(1)
                    .withSensorToMechanismRatio(1 / ANGLE_REDUCTION);
    public static final TalonFXConfiguration ANGLE_MOTOR_CONFIGS =
            new TalonFXConfiguration()
                    .withVoltage(VOLTAGE_CONFIGS)
                    .withCurrentLimits(TALON_FX_CURRENT_LIMIT_CONFIGS)
                    .withFeedback(FEEDBACK_CONFIGS_ANGLE)
                    .withMotorOutput(MOTOR_OUTPUT_CONFIGS);

    public static double DRIVE_MOTOR_MOMENT_OF_INERTIA = 0.025;
    public static double ANGLE_MOTOR_MOMENT_OF_INERTIA = 0.004;
    public static double MAX_X_Y_VELOCITY;
    public static double MAX_OMEGA_VELOCITY;

    public static void initConstants(boolean isWCP, boolean isReal) {
        if (!isReal) {
            DRIVE_KP.initDefault(2.0);
            DRIVE_KI.initDefault(0.0);
            DRIVE_KD.initDefault(0.0);
            DRIVE_KV.initDefault(0.0);
            DRIVE_KS.initDefault(0.0);
            DRIVE_KA.initDefault(0.0);

            ANGLE_KP.initDefault(12.0);
            ANGLE_KI.initDefault(0.0);
            ANGLE_KD.initDefault(0.0);
            ANGLE_KS.initDefault(0.0);

            ROTATION_KP.initDefault(0.9);
            ROTATION_KI.initDefault(0.0);
            ROTATION_KD.initDefault(0.0);
            ROTATION_KDIETER.initDefault(0.0);

            ROBOT_WIDTH = 0.584;
            ROBOT_LENGTH = 0.584;
            WHEEL_DIAMETER = 0.099;
            DRIVE_REDUCTION = (1 / 2.0) * (24.0 / 22.0) * (15.0 / 45.0);
            ANGLE_REDUCTION = (14.0 / 72.0) * 0.5;

            MAX_X_Y_VELOCITY =
                    6000.0
                            / 60.0
                            * // [m/s]
                            DRIVE_REDUCTION
                            * WHEEL_DIAMETER
                            * Math.PI;
        } else {
            if (isWCP) {
                DRIVE_KP.initDefault(0.0);
                DRIVE_KI.initDefault(0.0);
                DRIVE_KD.initDefault(0.0);
                DRIVE_KV.initDefault(0.6);
                DRIVE_KS.initDefault(0.6);
                DRIVE_KA.initDefault(0.0);

                ANGLE_KP.initDefault(28.0);
                ANGLE_KI.initDefault(0.0);
                ANGLE_KD.initDefault(0.0);
                ANGLE_KS.initDefault(0.28);

                ROTATION_KP.initDefault(0.9);
                ROTATION_KI.initDefault(0.0);
                ROTATION_KD.initDefault(0.0);
                ROTATION_KDIETER.initDefault(0.0);

                ROBOT_WIDTH = 0.584;
                ROBOT_LENGTH = 0.584;
                WHEEL_DIAMETER = 0.099;
                DRIVE_REDUCTION = (1 / 2.0) * (24.0 / 22.0) * (15.0 / 45.0);
                ANGLE_REDUCTION = (14.0 / 72.0) * 0.5;

                MAX_X_Y_VELOCITY =
                        6000.0
                                / 60.0
                                * // [m/s]
                                DRIVE_REDUCTION
                                * WHEEL_DIAMETER
                                * Math.PI;
            } else {
                DRIVE_KP.initDefault(0.0006);
                DRIVE_KI.initDefault(0.0);
                DRIVE_KD.initDefault(0.0);
                DRIVE_KV.initDefault(2.12);
                DRIVE_KS.initDefault(0.6);
                DRIVE_KA.initDefault(0.0);

                ANGLE_KP.initDefault(3.5);
                ANGLE_KI.initDefault(0.0);
                ANGLE_KD.initDefault(0.0);
                ANGLE_KS.initDefault(0.000_65);

                ROTATION_KP.initDefault(0.9);
                ROTATION_KI.initDefault(0.0);
                ROTATION_KD.initDefault(0.0);
                ROTATION_KDIETER.initDefault(0.0);

                ROBOT_WIDTH = 0.512;
                ROBOT_LENGTH = 0.67;
                WHEEL_DIAMETER = 0.099;
                DRIVE_REDUCTION = (12.0 / 24.0) * (28.0 / 20.0) * (15.0 / 45.0);
                ANGLE_REDUCTION = (6.0 / 40.0) * (11.0 / 59.0);

                MAX_X_Y_VELOCITY =
                        5676.0
                                / 60.0
                                * // [m/s]
                                DRIVE_REDUCTION
                                * WHEEL_DIAMETER
                                * Math.PI;
            }
        }

        MAX_OMEGA_VELOCITY =
                MAX_X_Y_VELOCITY
                        / // [m/s]
                        Math.sqrt(
                                (ROBOT_LENGTH / 2) * (ROBOT_LENGTH / 2)
                                        + (ROBOT_WIDTH / 2) * (ROBOT_WIDTH / 2));
        WHEEL_POSITIONS =
                new Translation2d[] {
                    new Translation2d(ROBOT_LENGTH / 2, ROBOT_WIDTH / 2), // FL
                    new Translation2d(ROBOT_LENGTH / 2, -ROBOT_WIDTH / 2), // FR
                    new Translation2d(-ROBOT_LENGTH / 2, ROBOT_WIDTH / 2), // RL
                    new Translation2d(-ROBOT_LENGTH / 2, -ROBOT_WIDTH / 2) // RR
                };
    }
}
