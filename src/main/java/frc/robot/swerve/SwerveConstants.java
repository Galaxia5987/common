package frc.robot.swerve;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
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

    public static final Slot0Configs DRIVE_PID_GAINS =
            new Slot0Configs().withKP(0.005).withKI(0.0).withKD(0.148).withKV(0.05).withKS(0);

    public static final Slot0Configs FRONT_LEFT_ANGLE_PID_GAINS =
            new Slot0Configs().withKP(1).withKI(0).withKD(0).withKV(0.2).withKS(0);

    public static final Slot0Configs FRONT_RIGHT_ANGLE_PID_GAINS =
            new Slot0Configs().withKP(1).withKI(0).withKD(0).withKV(0.2).withKS(0);

    public static final Slot0Configs REAR_LEFT_ANGLE_PID_GAINS =
            new Slot0Configs().withKP(1).withKI(0).withKD(0).withKV(0.2).withKS(0);

    public static final Slot0Configs REAR_RIGHT_ANGLE_PID_GAINS =
            new Slot0Configs().withKP(1).withKI(0).withKD(0).withKV(0.2).withKS(0);

    public static final MotionMagicConfigs MOTION_MAGIC_CONFIGS =
            new MotionMagicConfigs()
                    .withMotionMagicAcceleration(25000).withMotionMagicCruiseVelocity(21288).withMotionMagicJerk(1);

    public static final Slot0Configs[] SLOT_0_ANGLE_CONFIGS = {
            FRONT_LEFT_ANGLE_PID_GAINS,
            FRONT_RIGHT_ANGLE_PID_GAINS,
            REAR_LEFT_ANGLE_PID_GAINS,
            REAR_RIGHT_ANGLE_PID_GAINS
    };

    public static final double MAX_X_Y_VELOCITY = 6380.0 / 60.0 * //[m/s]
            DRIVE_REDUCTION *
            WHEEL_DIAMETER * Math.PI;

    public static final double MAX_OMEGA_VELOCITY = MAX_X_Y_VELOCITY / //[m/s]
            Math.sqrt((ROBOT_LENGTH / 2) * (ROBOT_LENGTH / 2) + (ROBOT_WIDTH / 2) * (ROBOT_WIDTH / 2));

    public static final TalonFXInvertType CLOCKWISE = TalonFXInvertType.Clockwise;
    public static final TalonFXInvertType COUNTER_CLOCKWISE = TalonFXInvertType.CounterClockwise;

    public static final double NEUTRAL_DEADBAND = 0.15;
    public static final double XBOX_DEADBAND = 0.15;

    public static final Translation2d[] WHEEL_POSITIONS = {
    public static final double TICKS_PER_RADIAN = FALCON_TICKS / ANGLE_REDUCTION / (Math.PI * 2);
    public static final double TICKS_PER_METER = (FALCON_TICKS / DRIVE_REDUCTION) / (Math.PI * WHEEL_DIAMETER);

            new Translation2d(ROBOT_LENGTH / 2, ROBOT_WIDTH / 2),   //FL
            new Translation2d(ROBOT_LENGTH / 2, -ROBOT_WIDTH / 2),   //FR
            new Translation2d(-ROBOT_LENGTH / 2, ROBOT_WIDTH / 2),  //RL
            new Translation2d(-ROBOT_LENGTH / 2, -ROBOT_WIDTH / 2)}; //RR

    public static final double MAX_VELOCITY_AUTO = 4.0;
    public static final double MAX_ACCELERATION_AUTO = 2.5;

    public static double AUTO_X_Kp = 3.20;
    public static double AUTO_X_Ki = 0.0;
    public static double AUTO_X_Kd = 0.73;
    public static double AUTO_Y_Kp = 3.20;
    public static double AUTO_Y_Ki = 0.0;
    public static double AUTO_Y_Kd = 0.6;
    public static double AUTO_ROTATION_Kp = 10;
    public static double AUTO_ROTATION_Ki = 0.0;
    public static double AUTO_ROTATION_Kd = 0.0;

    public static double FORWARD_BALANCE_TIME = 0.65;
    public static double BACKWARD_BALANCE_TIME = 0.55;
}
