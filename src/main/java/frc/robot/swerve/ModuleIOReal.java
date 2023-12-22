package frc.robot.swerve;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.utils.math.AngleUtil;
import frc.robot.utils.math.differential.Integral;
import frc.robot.utils.units.UnitModel;

public class ModuleIOReal implements ModuleIO {

    private final CANSparkMax driveMotor;
    private final CANSparkMax angleMotor;

    private final DutyCycleEncoder encoder;

    private final double[] motionMagicConfigs;
    private final UnitModel ticksPerRad = new UnitModel(SwerveConstants.TICKS_PER_RADIAN);
    private final UnitModel ticksPerMeter = new UnitModel(SwerveConstants.TICKS_PER_METER);
    private final int number;

    private double angleSetpoint;
    private double currentAngle;
    private double angleMotorPosition;
    private double driveMotorVelocitySetpoint;

    private final PIDController drivePIDController;
    private final PIDController anglePIDCOntroller;

    private final Integral driveSupplyChargeUsedCoulomb = new Integral(0, 0);
    private final Integral driveStatorChargeUsedCoulomb = new Integral(0, 0);

    private final Integral angleSupplyChargeUsedCoulomb = new Integral(0, 0);
    private final Integral angleStatorChargeUsedCoulomb = new Integral(0, 0);

    public ModuleIOReal(int driveMotorID, int angleMotorID, int encoderID,
                        boolean driveInverted, boolean angleInverted,
                        double[] motionMagicConfigs, int number) {

        this.driveMotor = new CANSparkMax(driveMotorID, CANSparkMaxLowLevel.MotorType.kBrushless);
        this.angleMotor = new CANSparkMax(angleMotorID, CANSparkMaxLowLevel.MotorType.kBrushless);

        this.encoder = new DutyCycleEncoder(encoderID);

        this.motionMagicConfigs = motionMagicConfigs;
        this.number = number;

        driveMotor.restoreFactoryDefaults();
        angleMotor.restoreFactoryDefaults();

        driveMotor.getPIDController().setP(SwerveConstants.DRIVE_kP);
        driveMotor.getPIDController().setI(SwerveConstants.DRIVE_kI);
        driveMotor.getPIDController().setD(SwerveConstants.DRIVE_kD);
        driveMotor.getPIDController().setFF(SwerveConstants.DRIVE_KF);
        driveMotor.enableVoltageCompensation(SwerveConstants.VOLT_COMP_SATURATION);
        driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        driveMotor.setSmartCurrentLimit(SwerveConstants.CURRENT_LIMIT);
        driveMotor.setInverted(driveInverted);
        driveMotor.burnFlash();

        angleMotor.enableVoltageCompensation(SwerveConstants.VOLT_COMP_SATURATION);
        angleMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        angleMotor.setSmartCurrentLimit(SwerveConstants.ALT_CURRENT_LIMIT);
        angleMotor.setInverted(angleInverted);

        angleMotor.getPIDController().setP(motionMagicConfigs[0]);
        angleMotor.getPIDController().setI(motionMagicConfigs[1]);
        angleMotor.getPIDController().setD(motionMagicConfigs[2]);
        angleMotor.getPIDController().setFF(motionMagicConfigs[3]);
        angleMotor.getPIDController().setSmartMotionMaxVelocity(motionMagicConfigs[5], 0);
        angleMotor.getPIDController().setSmartMotionMaxAccel(motionMagicConfigs[6], 0);
        angleMotor.getPIDController().setSmartMotionAllowedClosedLoopError(motionMagicConfigs[7], 0);
        angleMotor.getPIDController().setIMaxAccum(motionMagicConfigs[8], 0);
        angleMotor.getPIDController().setOutputRange(-motionMagicConfigs[9], motionMagicConfigs[9]);
        angleMotor.burnFlash();

        drivePIDController = new PIDController(SwerveConstants.DRIVE_kP, SwerveConstants.DRIVE_kI, SwerveConstants.DRIVE_kD);
        anglePIDCOntroller = new PIDController(motionMagicConfigs[0], motionMagicConfigs[1], motionMagicConfigs[2]);
    }

    @Override
    public void updateInputs(SwerveModuleInputs inputs) {
        inputs.absolutePosition = encoder.getAbsolutePosition();

        inputs.driveMotorSupplyCurrent = driveMotor.getOutputCurrent();
        driveSupplyChargeUsedCoulomb.update(inputs.driveMotorSupplyCurrent);
        inputs.driveMotorSupplyCurrentOverTime = driveSupplyChargeUsedCoulomb.get();
        driveStatorChargeUsedCoulomb.update(inputs.driveMotorStatorCurrent);
        inputs.driveMotorStatorCurrentOverTime = driveStatorChargeUsedCoulomb.get();
        inputs.driveMotorPosition = driveMotor.getEncoder().getPosition() * 42;
        inputs.driveMotorVelocity = getVelocity();
        inputs.driveMotorVelocitySetpoint = driveMotorVelocitySetpoint;

        inputs.angleMotorSupplyCurrent = angleMotor.getOutputCurrent();
        angleSupplyChargeUsedCoulomb.update(inputs.angleMotorSupplyCurrent);
        inputs.angleMotorSupplyCurrentOverTime = angleSupplyChargeUsedCoulomb.get();
        angleStatorChargeUsedCoulomb.update(inputs.angleMotorStatorCurrent);
        inputs.angleMotorStatorCurrentOverTime = angleStatorChargeUsedCoulomb.get();
        inputs.angleMotorPosition = angleMotor.getEncoder().getPosition() * 42;
        angleMotorPosition = inputs.angleMotorPosition;
        inputs.angleMotorVelocity = ticksPerMeter.toVelocity((angleMotor.getEncoder().getVelocity() / 60) * 42);

        inputs.angle = getAngle();
        currentAngle = inputs.angle;

        inputs.angleSetpoint = angleSetpoint;

        inputs.moduleDistance = getModulePosition().distanceMeters;
    }

    @Override
    public double getAngle() {
        return AngleUtil.normalize(ticksPerRad.toUnits(angleMotor.getEncoder().getPosition()));
    }

    @Override
    public void setAngle(double angle) {
        angleSetpoint = AngleUtil.normalize(angle);
        Rotation2d error = new Rotation2d(angle).minus(new Rotation2d(currentAngle));
        double output = anglePIDCOntroller.calculate(angleMotorPosition, angleMotorPosition + ticksPerRad.toTicks(error.getRadians()));
        angleMotor.set(output);
    }

    @Override
    public double getVelocity() {
        return ticksPerMeter.toVelocity((driveMotor.getEncoder().getVelocity() / 60) * 42);
    }

    @Override
    public void setVelocity(double velocity) {
        var angleError = new Rotation2d(angleSetpoint).minus(new Rotation2d(currentAngle));
        velocity *= angleError.getCos();
        driveMotorVelocitySetpoint = velocity;
        double output = drivePIDController.calculate(getVelocity(), driveMotorVelocitySetpoint);
        driveMotor.set(output);
    }

    @Override
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(
                ticksPerMeter.toUnits(driveMotor.getEncoder().getPosition() * 42),
                new Rotation2d(getAngle())
        );
    }

    @Override
    public void updateOffset(double offset) {
        angleMotor.getEncoder().setPosition(
                (((encoder.getAbsolutePosition() - offset) * 42) / SwerveConstants.ANGLE_REDUCTION) / 42);
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
}
