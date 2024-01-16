package frc.robot.swerve;

import com.ctre.phoenix6.configs.*;
import edu.wpi.first.math.geometry.Translation2d;
import lib.webconstants.LoggedTunableNumber;

public class SwerveConstantsTalonFX {
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

    public static final LoggedTunableNumber DRIVE_KP = new LoggedTunableNumber("Swerve Drive/PID/driveKP");
    public static final LoggedTunableNumber DRIVE_KI = new LoggedTunableNumber("Swerve Drive/PID/driveKI");
    public static final LoggedTunableNumber DRIVE_KD = new LoggedTunableNumber("Swerve Drive/PID/driveKD");
    public static final LoggedTunableNumber DRIVE_KV = new LoggedTunableNumber("Swerve Drive/PID/driveKV");
    public static final LoggedTunableNumber DRIVE_KS = new LoggedTunableNumber("Swerve Drive/PID/driveKS");

    public static final LoggedTunableNumber ANGLE_KP = new LoggedTunableNumber("Swerve Drive/PID/angleKP");
    public static final LoggedTunableNumber ANGLE_KI = new LoggedTunableNumber("Swerve Drive/PID/angleKI");
    public static final LoggedTunableNumber ANGLE_KD = new LoggedTunableNumber("Swerve Drive/PID/angleKD");
    public static final LoggedTunableNumber ANGLE_KS = new LoggedTunableNumber("Swerve Drive/PID/angleKS");

    public static final LoggedTunableNumber[] PID_VALUES = new LoggedTunableNumber[]{DRIVE_KP, DRIVE_KI, DRIVE_KD, DRIVE_KV, DRIVE_KS, ANGLE_KP, ANGLE_KI, ANGLE_KD, ANGLE_KS};

    public static void initConstants(boolean isWCP, boolean isReal){
        if (!isReal){
            DRIVE_KP.initDefault(3.5);
            DRIVE_KI.initDefault(0.0);
            DRIVE_KD.initDefault(0.0);

            ANGLE_KP.initDefault(8.0);
            ANGLE_KI.initDefault(0.0);
            ANGLE_KD.initDefault(0.0);
        }
        else{
            if (isWCP){
                DRIVE_KP.initDefault(0.0);
                DRIVE_KI.initDefault(0.0);
                DRIVE_KD.initDefault(0.0);
                DRIVE_KV.initDefault(0.6);
                DRIVE_KS.initDefault(0.6);

                ANGLE_KP.initDefault(28.0);
                ANGLE_KI.initDefault(0.0);
                ANGLE_KD.initDefault(0.0);
                ANGLE_KS.initDefault(0.28);
            }
            else{
                DRIVE_KP.initDefault(0.0006);
                DRIVE_KI.initDefault(0.0);
                DRIVE_KD.initDefault(10);
                DRIVE_KV.initDefault(2.12);
                DRIVE_KS.initDefault(0.6);

                ANGLE_KP.initDefault(3.5);
                ANGLE_KI.initDefault(0.0);
                ANGLE_KD.initDefault(0.0);
                ANGLE_KS.initDefault(0.00065);
            }
        }
    }

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

    public static final double ODOMETRY_FREQUENCY = 250;
}
