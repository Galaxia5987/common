package frc.robot.swerve;

import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import utils.math.AngleUtil;
import utils.math.differential.Integral;
import utils.motors.TalonFXSim;
import utils.units.Units;

public class ModuleIOSim implements ModuleIO {
    private final TalonFXSim driveMotor;
    private final TalonFXSim angleMotor;

    private VoltageOut driveControl = new VoltageOut(0);
    private VoltageOut angleControl = new VoltageOut(0);

    private final PIDController angleFeedback;
    private final PIDController velocityFeedback;
    private final Integral currentAngle = new Integral();
    private final Integral moduleDistance = new Integral();
    private double currentVelocity = 0;
    private double driveMotorAppliedVoltage = 0;
    private double angleMotorAppliedVoltage = 0;
    private double velocitySetpoint = 0;
    private double angleSetpoint = 0;

    public ModuleIOSim() {
        driveMotor = new TalonFXSim(
                1,
                1/SwerveConstants.DRIVE_REDUCTION,
                SwerveConstants.DRIVE_MOTOR_MOMENT_OF_INERTIA);

        angleMotor = new TalonFXSim(
                1,
                1 / SwerveConstants.ANGLE_REDUCTION,
                SwerveConstants.ANGLE_MOTOR_MOMENT_OF_INERTIA);

        angleFeedback = new PIDController(3.5, 0, 0, 0.02);
        velocityFeedback = new PIDController(0.5, 0, 0.00, 0.02);
    }

    @Override
    public void updateInputs(SwerveModuleInputs inputs) {
        driveMotor.update(Timer.getFPGATimestamp());
        angleMotor.update(Timer.getFPGATimestamp());

        currentAngle.update(Units.rpsToRadsPerSec(angleMotor.getRotorVelocity()));

        inputs.driveMotorAppliedVoltage = driveMotorAppliedVoltage;
        inputs.driveMotorVelocity = driveMotor.getRotorVelocity();
        inputs.driveMotorVelocitySetpoint = velocitySetpoint;

        inputs.angleMotorAppliedVoltage = angleMotorAppliedVoltage;
        inputs.angleMotorVelocity = angleMotor.getRotorVelocity();
        inputs.angleSetpoint = angleSetpoint;
        inputs.angle = AngleUtil.normalize(currentAngle.get());

        moduleDistance.update(inputs.driveMotorVelocity);
        inputs.moduleDistance = moduleDistance.get();

        driveMotor.setControl(driveControl);
        angleMotor.setControl(angleControl);
    }

    @Override
    public double getAngle() {
        return currentAngle.get();
    }

    @Override
    public void setAngle(Rotation2d angle) {
        angleSetpoint = angle.getRadians();
        angleMotorAppliedVoltage = angleFeedback.calculate(MathUtil.angleModulus(currentAngle.get()), angleSetpoint);
        angleControl.withOutput(angleMotorAppliedVoltage);
    }

    @Override
    public double getVelocity() {
        return currentVelocity;
    }

    @Override
    public void setVelocity(double velocity) {
        var angleError = new Rotation2d(angleSetpoint).minus(new Rotation2d(currentAngle.get()));
        velocity *= angleError.getCos();

        velocitySetpoint = velocity;
        currentVelocity = Units.rpsToMetersPerSecond(driveMotor.getVelocity(1/SwerveConstants.DRIVE_REDUCTION), SwerveConstants.WHEEL_DIAMETER/2);
        driveMotorAppliedVoltage = velocityFeedback.calculate(currentVelocity, velocity);
        driveControl.withOutput(driveMotorAppliedVoltage);
    }
}
