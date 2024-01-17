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

    private VelocityVoltage driveControlRequest = new VelocityVoltage(0).withEnableFOC(true);
    private PositionVoltage angleControlRequest = new PositionVoltage(0).withEnableFOC(true);

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

        velocityController =
                new PIDController(
                        SwerveConstants.DRIVE_KP.get(),
                        SwerveConstants.DRIVE_KI.get(),
                        SwerveConstants.DRIVE_KD.get(),
                        0.02);
        angleController =
                new PIDController(
                        SwerveConstants.ANGLE_KP.get(),
                        SwerveConstants.ANGLE_KI.get(),
                        SwerveConstants.ANGLE_KD.get(),
                        0.02);

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
        inputs.angle = AngleUtil.normalize(currentAngle);

        inputs.moduleDistance =
                Units.rpsToMetersPerSecond(
                        driveMotor.getRotorPosition(), SwerveConstants.WHEEL_DIAMETER / 2);
        inputs.moduleState = getModuleState();

        if (hasPIDChanged(SwerveConstants.PID_VALUES)) updatePID();
    }

    @Override
    public void updatePID() {
        velocityController.setPID(
                SwerveConstants.DRIVE_KP.get(),
                SwerveConstants.DRIVE_KI.get(),
                SwerveConstants.DRIVE_KD.get());
        angleController.setPID(
                SwerveConstants.ANGLE_KP.get(),
                SwerveConstants.ANGLE_KI.get(),
                SwerveConstants.ANGLE_KD.get());
    }

    @Override
    public Rotation2d getAngle() {
        return currentAngle;
    }

    @Override
    public void setAngle(Rotation2d angle) {
        angleSetpoint = angle;
        angleMotor.setControl(angleControlRequest.withPosition(angle.getRotations()));
    }

    @Override
    public double getVelocity() {
        return currentVelocity;
    }

    @Override
    public void setVelocity(double velocity) {
        velocitySetpoint = velocity;
        driveControlRequest.withVelocity(
                Units.metersToRotations(velocity, SwerveConstants.WHEEL_DIAMETER / 2));
        driveMotor.setControl(driveControlRequest);
    }

    @Override
    public void setAngleVelocity(double velocity) {
        angleControlRequest.withVelocity(velocity);
        angleMotor.setControl(angleControlRequest);
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
        driveControlRequest.withVelocity(0);
        driveMotor.setControl(driveControlRequest);

        angleControlRequest.withVelocity(0);
        angleMotor.setControl(angleControlRequest);
    }
}
