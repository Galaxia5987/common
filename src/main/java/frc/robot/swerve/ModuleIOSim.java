package frc.robot.swerve;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import lib.motors.TalonFXSim;
import lib.units.Units;

public class ModuleIOSim implements ModuleIO {
    private final TalonFXSim driveMotor;
    private final TalonFXSim angleMotor;

    private VelocityVoltage driveControl = new VelocityVoltage(0).withEnableFOC(true);
    private PositionVoltage angleControl = new PositionVoltage(0).withEnableFOC(true);

    private final PIDController angleController;
    private final PIDController velocityController;
    private double currentVelocity = 0;
    private double velocitySetpoint = 0;
    private Rotation2d currentAngle = new Rotation2d();
    private Rotation2d angleSetpoint = new Rotation2d();

    public ModuleIOSim() {
        driveMotor =
                new TalonFXSim(
                        1,
                        1 / SwerveConstants.DRIVE_REDUCTION,
                        SwerveConstants.DRIVE_MOTOR_MOMENT_OF_INERTIA);

        angleMotor =
                new TalonFXSim(
                        1,
                        1 / SwerveConstants.ANGLE_REDUCTION,
                        SwerveConstants.ANGLE_MOTOR_MOMENT_OF_INERTIA);

        angleController = new PIDController(8, 0, 0, 0.02);
        velocityController = new PIDController(3.5, 0, 0.00, 0.02);

        driveMotor.setController(velocityController);
        angleMotor.setController(angleController);
    }

    @Override
    public void updateInputs(SwerveModuleInputs inputs) {
        driveMotor.update(Timer.getFPGATimestamp());
        angleMotor.update(Timer.getFPGATimestamp());

        inputs.driveMotorAppliedVoltage = driveMotor.getAppliedVoltage();
        inputs.driveMotorVelocity =
                Units.rpsToMetersPerSecond(
                        driveMotor.getRotorVelocity(), SwerveConstants.WHEEL_DIAMETER / 2);
        currentVelocity = inputs.driveMotorVelocity;
        inputs.driveMotorVelocitySetpoint = velocitySetpoint;

        inputs.angleMotorAppliedVoltage = angleMotor.getAppliedVoltage();
        inputs.angleMotorVelocity = angleMotor.getRotorVelocity();
        inputs.angleSetpoint = angleSetpoint;
        inputs.angle = Rotation2d.fromRotations(angleMotor.getRotorPosition());
        currentAngle = inputs.angle;

        inputs.moduleDistance = Units.rpsToMetersPerSecond(driveMotor.getRotorPosition(), SwerveConstants.WHEEL_DIAMETER/2);
        inputs.moduleState = getModuleState();
    }

    @Override
    public Rotation2d getAngle() {
        return currentAngle;
    }

    @Override
    public void setAngle(Rotation2d angle) {
        angleSetpoint = angle;
        angleMotor.setControl(angleControl.withPosition(angle.getRotations()));
    }

    @Override
    public double getVelocity() {
        return currentVelocity;
    }

    @Override
    public void setVelocity(double velocity) {
        velocitySetpoint = velocity;
        driveControl.withVelocity(
                Units.metersToRotations(velocity, SwerveConstants.WHEEL_DIAMETER / 2));
        driveMotor.setControl(driveControl);
    }

    @Override
    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(getVelocity(), getAngle());
    }

    @Override
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(
                Units.rpsToMetersPerSecond(
                        driveMotor.getRotorPosition(), SwerveConstants.WHEEL_DIAMETER / 2),
                currentAngle);
    }

    @Override
    public void stop() {
        driveControl.withVelocity(0);
        driveMotor.setControl(driveControl);

        angleControl.withVelocity(0);
        angleMotor.setControl(angleControl);
    }

    @Override
    public void checkModule() {
        driveControl.withVelocity(0.8*SwerveConstants.MAX_X_Y_VELOCITY);
        angleControl.withVelocity(0.2*SwerveConstants.MAX_OMEGA_VELOCITY);
    }
}
