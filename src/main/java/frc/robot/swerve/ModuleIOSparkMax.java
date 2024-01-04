package frc.robot.swerve;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotController;
import lib.math.AngleUtil;
import lib.math.differential.Integral;
import lib.units.Units;

public class ModuleIOSparkMax implements ModuleIO {

    private final CANSparkMax driveMotor;
    private final SparkMaxPIDController drivePIDController;
    private final RelativeEncoder driveEncoder;
    private final CANSparkMax angleMotor;
    private final SparkMaxPIDController anglePIDController;
    private final RelativeEncoder angleEncoder;

    private final DutyCycleEncoder encoder;

    private final Integral driveSupplyChargeUsedCoulomb = new Integral();
    private final Integral driveStatorChargeUsedCoulomb = new Integral();
    private final Integral angleSupplyChargeUsedCoulomb = new Integral();
    private final Integral angleStatorChargeUsedCoulomb = new Integral();
    private double angleSetpoint;
    private double currentAngle;
    private double angleMotorPosition;
    private double moduleDistance;
    private double driveMotorSetpoint;

    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
            SwerveConstants.DRIVE_kS,
            SwerveConstants.DRIVE_kV,
            SwerveConstants.DRIVE_kA
    );

    public ModuleIOSparkMax(int driveMotorID, int angleMotorID, int encoderID,
                            boolean driveInverted, boolean angleInverted,
                            double[] motionMagicConfigs) {

        this.driveMotor = new CANSparkMax(driveMotorID, CANSparkMaxLowLevel.MotorType.kBrushless);
        this.angleMotor = new CANSparkMax(angleMotorID, CANSparkMaxLowLevel.MotorType.kBrushless);

        this.encoder = new DutyCycleEncoder(encoderID);

        driveMotor.restoreFactoryDefaults();
        drivePIDController = driveMotor.getPIDController();
        driveEncoder = driveMotor.getEncoder();

        driveMotor.enableVoltageCompensation(SwerveConstants.VOLT_COMP_SATURATION);
        driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        driveMotor.setSmartCurrentLimit(SwerveConstants.CURRENT_LIMIT);
        driveMotor.setInverted(driveInverted);
        driveEncoder.setPositionConversionFactor(SwerveConstants.DRIVE_REDUCTION);
        driveEncoder.setVelocityConversionFactor(SwerveConstants.DRIVE_REDUCTION);
        driveMotor.burnFlash();

        angleMotor.restoreFactoryDefaults();
        anglePIDController = angleMotor.getPIDController();
        angleEncoder = angleMotor.getEncoder();

        angleMotor.enableVoltageCompensation(SwerveConstants.VOLT_COMP_SATURATION);
        angleMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        angleMotor.setSmartCurrentLimit(SwerveConstants.ALT_CURRENT_LIMIT);
        angleMotor.setInverted(angleInverted);
        anglePIDController.setP(motionMagicConfigs[0]);
        anglePIDController.setI(motionMagicConfigs[1]);
        anglePIDController.setD(motionMagicConfigs[2]);
        anglePIDController.setFF(motionMagicConfigs[3]);
        anglePIDController.setSmartMotionMaxVelocity(motionMagicConfigs[5], 0);
        anglePIDController.setSmartMotionMaxAccel(motionMagicConfigs[6], 0);
//        anglePIDController.setSmartMotionAllowedClosedLoopError(motionMagicConfigs[7], 0);
//        anglePIDController.setIMaxAccum(motionMagicConfigs[8], 0);
//        anglePIDController.setOutputRange(-motionMagicConfigs[9], motionMagicConfigs[9]);
        angleEncoder.setPositionConversionFactor(SwerveConstants.ANGLE_REDUCTION);
        angleEncoder.setVelocityConversionFactor(SwerveConstants.ANGLE_REDUCTION);
        angleMotor.burnFlash();
    }

    @Override
    public void updateInputs(SwerveModuleInputs inputs) {
        inputs.absolutePosition = getEncoderAngle();

        inputs.driveMotorSupplyCurrent = driveMotor.getOutputCurrent();
        driveSupplyChargeUsedCoulomb.update(inputs.driveMotorSupplyCurrent);
        inputs.driveMotorSupplyCurrentOverTime = driveSupplyChargeUsedCoulomb.get();
        driveStatorChargeUsedCoulomb.update(inputs.driveMotorStatorCurrent);
        inputs.driveMotorStatorCurrentOverTime = driveStatorChargeUsedCoulomb.get();
        inputs.driveMotorPosition = driveEncoder.getPosition();
        inputs.driveMotorVelocity = getVelocity();
        inputs.driveMotorVelocitySetpoint = driveMotorSetpoint;
        inputs.driveMotorAppliedVoltage = driveMotor.getAppliedOutput() * RobotController.getBatteryVoltage();

        inputs.angleMotorSupplyCurrent = angleMotor.getOutputCurrent();
        angleSupplyChargeUsedCoulomb.update(inputs.angleMotorSupplyCurrent);
        inputs.angleMotorSupplyCurrentOverTime = angleSupplyChargeUsedCoulomb.get();
        angleStatorChargeUsedCoulomb.update(inputs.angleMotorStatorCurrent);
        inputs.angleMotorStatorCurrentOverTime = angleStatorChargeUsedCoulomb.get();
        inputs.angleMotorPosition = angleEncoder.getPosition();
        angleMotorPosition = inputs.angleMotorPosition;
        inputs.angleMotorVelocity = Units.rpmToRps(angleEncoder.getVelocity());

        inputs.angle = AngleUtil.normalize(angleEncoder.getPosition() * 2 * Math.PI);
        currentAngle = inputs.angle;

        inputs.angleSetpoint = angleSetpoint;

        inputs.moduleDistance = inputs.driveMotorPosition * SwerveConstants.WHEEL_CIRCUMFERENCE;
        moduleDistance = inputs.moduleDistance;
    }

    @Override
    public double getAngle() {
        return currentAngle;
    }

    @Override
    public void setAngle(double angle) {
        angleSetpoint = AngleUtil.normalize(angle);
        Rotation2d error = new Rotation2d(angle).minus(new Rotation2d(currentAngle));
        anglePIDController.setReference(
                angleMotorPosition + error.getRotations(),
                CANSparkMax.ControlType.kPosition);
    }

    @Override
    public double getVelocity() {
        return Units.rpmToRadsPerSec(driveEncoder.getVelocity()) * (SwerveConstants.WHEEL_DIAMETER / 2);
    }

    @Override
    public void setVelocity(double velocity) {
        var angleError = new Rotation2d(angleSetpoint).minus(new Rotation2d(currentAngle));
        velocity *= angleError.getCos();
        driveMotorSetpoint = velocity;
        drivePIDController.setReference(
                feedforward.calculate(velocity),
                CANSparkMax.ControlType.kVoltage);
    }

    @Override
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(
                moduleDistance,
                new Rotation2d(getAngle())
        );
    }

    @Override
    public void updateOffset(double offset) {
        angleEncoder.setPosition(getEncoderAngle() - offset);
    }

    @Override
    public void stop() {
        driveMotor.stopMotor();
        angleMotor.stopMotor();
    }

    @Override
    public boolean encoderConnected() {
        return encoder.isConnected();
    }

    @Override
    public void checkModule() {
        driveMotor.set(0.8);
        angleMotor.set(0.2);
    }

    private double getEncoderAngle() {
        return 1.0 - encoder.getAbsolutePosition();
    }
}
