package frc.robot.swerve;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import utils.math.AngleUtil;
import utils.math.differential.Integral;
import utils.motors.TalonFXSim;
import utils.units.Units;

public class ModuleIOSim implements ModuleIO {
    private final TalonFXSim driveMotor;
    private final TalonFXSim angleMotor;

    private VelocityVoltage driveControl = new VelocityVoltage(0).withEnableFOC(true);
    private PositionVoltage angleControl = new PositionVoltage(0).withEnableFOC(true);

    private final PIDController angleFeedback;
    private final PIDController velocityFeedback;
    private final Integral moduleDistance = new Integral();
    private double currentVelocity = 0;
    private double velocitySetpoint = 0;
    private Rotation2d currentAngle = new Rotation2d();
    private Rotation2d angleSetpoint = new Rotation2d();

    public ModuleIOSim() {
        driveMotor = new TalonFXSim(
                1,
                1/SwerveConstants.DRIVE_REDUCTION,
                SwerveConstants.DRIVE_MOTOR_MOMENT_OF_INERTIA);

        angleMotor = new TalonFXSim(
                1,
                1 / SwerveConstants.ANGLE_REDUCTION,
                SwerveConstants.ANGLE_MOTOR_MOMENT_OF_INERTIA);

        angleFeedback = new PIDController(8, 0, 0, 0.02);
        velocityFeedback = new PIDController(3.5, 0, 0.00, 0.02);

        driveMotor.setController(velocityFeedback);
        angleMotor.setController(angleFeedback);
    }

    @Override
    public void updateInputs(SwerveModuleInputs inputs) {
        driveMotor.update(Timer.getFPGATimestamp());
        angleMotor.update(Timer.getFPGATimestamp());

        inputs.driveMotorAppliedVoltage = driveMotor.getAppliedVoltage();
        inputs.driveMotorVelocity = Units.rpsToMetersPerSecond(
                driveMotor.getRotorVelocity(),
                SwerveConstants.WHEEL_DIAMETER/2);
        currentVelocity = inputs.driveMotorVelocity;
        inputs.driveMotorVelocitySetpoint = velocitySetpoint;

        inputs.angleMotorAppliedVoltage = angleMotor.getAppliedVoltage();
        inputs.angleMotorVelocity = angleMotor.getRotorVelocity();
        inputs.angleSetpoint = angleSetpoint;
        inputs.angle = Rotation2d.fromRotations(angleMotor.getRotorPosition());
        currentAngle = inputs.angle;

        moduleDistance.update(inputs.driveMotorVelocity);
        inputs.moduleDistance = moduleDistance.get();
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
        var angleError = angleSetpoint.minus(currentAngle);
        velocity *= angleError.getCos();

        velocitySetpoint = velocity;
        driveControl.withVelocity(Units.metersToRotations(velocity, SwerveConstants.WHEEL_DIAMETER/2));
        driveMotor.setControl(driveControl);
    }

    @Override
    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(
                getVelocity(),
                getAngle()
        );
    }

    @Override
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(Units.rpsToMetersPerSecond(
                driveMotor.getRotorPosition(), SwerveConstants.WHEEL_DIAMETER/2),
                currentAngle);
    }

    @Override
    public void stopMotor() {
        driveControl.withVelocity(0);
        driveMotor.setControl(driveControl);

        angleControl.withVelocity(0);
        angleMotor.setControl(angleControl);
    }
}
