package frc.robot.swerve;

import com.ctre.phoenix6.configs.*;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;

public class SwerveConstants {
    public static final double[] OFFSETS =
            {0.5686593642164841, 0.26232458155811456, 0.00854002521350063, 0.5429330635733266};

    public static final double VOLT_COMP_SATURATION = 12;
    public static final CurrentLimitsConfigs CURRENT_LIMITS_CONFIGS = new
            CurrentLimitsConfigs().withSupplyCurrentLimit(50).withStatorCurrentLimit(50);
    public static final double ROBOT_WIDTH = 0.512; //[m]
    public static final double ROBOT_LENGTH = 0.67; //[m]
    public static final double WHEEL_DIAMETER = 0.1023679821; //[m]
    public static final double DRIVE_REDUCTION = (1 / 2.0) * (24.0 / 22.0) * (15.0 / 45.0);
    public static final double ANGLE_REDUCTION = (14.0 / 72.0) * 0.5;
    public static final double DRIVE_MOTOR_MOMENT_OF_INERTIA = 0.025;
    public static final double ANGLE_MOTOR_MOMENT_OF_INERTIA = 0.004;

    public static final double NEUTRAL_DEADBAND = 0.15;
    public static final double XBOX_DEADBAND = 0.15;

    public static final Slot0Configs DRIVE_PID_GAINS =
            new Slot0Configs().withKP(0.0).withKI(0.0).withKD(0.0).withKV(2.12).withKS(0.6);
    public static final Slot0Configs ANGLE_PID_GAINS =
            new Slot0Configs().withKP(1).withKI(0).withKD(0).withKV(0.2).withKS(0);
    public static final MotionMagicConfigs MOTION_MAGIC_CONFIGS =
            new MotionMagicConfigs().withMotionMagicAcceleration(25000).withMotionMagicCruiseVelocity(21288).withMotionMagicJerk(1);

    public static final VoltageConfigs VOLTAGE_CONFIGS = new VoltageConfigs().withPeakForwardVoltage(VOLT_COMP_SATURATION).withPeakReverseVoltage(VOLT_COMP_SATURATION);
    public static final FeedbackConfigs FEEDBACK_CONFIGS = new FeedbackConfigs().withRotorToSensorRatio(1).withSensorToMechanismRatio(1/DRIVE_REDUCTION);
    public static final MotorOutputConfigs MOTOR_OUTPUT_CONFIGS = new MotorOutputConfigs().withDutyCycleNeutralDeadband(NEUTRAL_DEADBAND);

    public static final TalonFXConfiguration DRIVE_MOTOR_CONFIGS = new TalonFXConfiguration()
            .withSlot0(DRIVE_PID_GAINS).withVoltage(VOLTAGE_CONFIGS)
            .withCurrentLimits(CURRENT_LIMITS_CONFIGS).withFeedback(FEEDBACK_CONFIGS);
    public static final TalonFXConfiguration ANGLE_MOTOR_CONFIGS = new TalonFXConfiguration()
            .withSlot0(ANGLE_PID_GAINS).withMotionMagic(MOTION_MAGIC_CONFIGS).withVoltage(VOLTAGE_CONFIGS)
            .withCurrentLimits(CURRENT_LIMITS_CONFIGS).withFeedback(FEEDBACK_CONFIGS).withMotorOutput(MOTOR_OUTPUT_CONFIGS);

    public static final double MAX_X_Y_VELOCITY = 6380.0 / 60.0 * //[m/s]
            DRIVE_REDUCTION *
            WHEEL_DIAMETER * Math.PI;

    public static final double MAX_OMEGA_VELOCITY = MAX_X_Y_VELOCITY / //[m/s]
            Math.sqrt((ROBOT_LENGTH / 2) * (ROBOT_LENGTH / 2) + (ROBOT_WIDTH / 2) * (ROBOT_WIDTH / 2));

    public static final Translation2d[] WHEEL_POSITIONS = {
            new Translation2d(ROBOT_LENGTH / 2, ROBOT_WIDTH / 2),   //FL
            new Translation2d(ROBOT_LENGTH / 2, -ROBOT_WIDTH / 2),   //FR
            new Translation2d(-ROBOT_LENGTH / 2, ROBOT_WIDTH / 2),  //RL
            new Translation2d(-ROBOT_LENGTH / 2, -ROBOT_WIDTH / 2)}; //RR
}
