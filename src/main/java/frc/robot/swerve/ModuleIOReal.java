package frc.robot.swerve;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import lib.math.AngleUtil;
import lib.math.differential.Integral;
import lib.units.UnitModel;

public class ModuleIOReal implements ModuleIO {

    private final TalonFX driveMotor;
    private final TalonFX angleMotor;

    private final DutyCycleEncoder encoder;

    private final double[] motionMagicConfigs;
    private final UnitModel ticksPerRad = new UnitModel(SwerveConstants.TICKS_PER_RADIAN);
    private final UnitModel ticksPerMeter = new UnitModel(SwerveConstants.TICKS_PER_METER);
    private final int number;
    private final Integral driveSupplyChargeUsedCoulomb = new Integral();
    private final Integral driveStatorChargeUsedCoulomb = new Integral();
    private final Integral angleSupplyChargeUsedCoulomb = new Integral();
    private final Integral angleStatorChargeUsedCoulomb = new Integral();
    private double angleSetpoint;
    private double currentAngle;
    private double angleMotorPosition;
    private double driveMotorVelocitySetpoint;

    public ModuleIOReal(int driveMotorID, int angleMotorID, int encoderID,
                        double[] motionMagicConfigs, int number) {

        this.driveMotor = new TalonFX(driveMotorID);
        this.angleMotor = new TalonFX(angleMotorID);

        this.encoder = new DutyCycleEncoder(encoderID);

        this.motionMagicConfigs = motionMagicConfigs;
        this.number = number;

        driveMotor.configFactoryDefault(100);
        angleMotor.configFactoryDefault(100);

        driveMotor.config_kP(0, SwerveConstants.DRIVE_kP, 100);
        driveMotor.config_kI(0, SwerveConstants.DRIVE_kI, 100);
        driveMotor.config_kD(0, SwerveConstants.DRIVE_kD, 100);
        driveMotor.config_kF(0, SwerveConstants.DRIVE_KF, 100);
        driveMotor.enableVoltageCompensation(true);
        driveMotor.configVoltageCompSaturation(SwerveConstants.VOLT_COMP_SATURATION);
        driveMotor.setNeutralMode(NeutralMode.Brake);
        driveMotor.configSupplyCurrentLimit(SwerveConstants.SUPPLY_CURRENT_LIMIT);
        driveMotor.configStatorCurrentLimit(SwerveConstants.STATOR_CURRENT_LIMIT);
        driveMotor.setInverted(SwerveConstants.CLOCKWISE);

        angleMotor.enableVoltageCompensation(true);
        angleMotor.configVoltageCompSaturation(SwerveConstants.VOLT_COMP_SATURATION);
        angleMotor.configNeutralDeadband(SwerveConstants.NEUTRAL_DEADBAND);
        angleMotor.setNeutralMode(NeutralMode.Brake);
        angleMotor.configSupplyCurrentLimit(SwerveConstants.SUPPLY_CURRENT_LIMIT);
        angleMotor.configStatorCurrentLimit(SwerveConstants.STATOR_CURRENT_LIMIT);
        angleMotor.setInverted(SwerveConstants.CLOCKWISE);

        angleMotor.config_kP(0, motionMagicConfigs[0], 100);
        angleMotor.config_kI(0, motionMagicConfigs[1], 100);
        angleMotor.config_kD(0, motionMagicConfigs[2], 100);
        angleMotor.config_kF(0, motionMagicConfigs[3], 100);
        angleMotor.configMotionSCurveStrength((int) motionMagicConfigs[4], 100);
        angleMotor.configMotionCruiseVelocity(motionMagicConfigs[5], 100);
        angleMotor.configMotionAcceleration(motionMagicConfigs[6], 100);
        angleMotor.configAllowableClosedloopError(0, motionMagicConfigs[7], 100);
        angleMotor.configMaxIntegralAccumulator(0, motionMagicConfigs[8], 100);
        angleMotor.configClosedLoopPeakOutput(0, motionMagicConfigs[9], 100);
    }

    @Override
    public void updateInputs(SwerveModuleInputs inputs) {
        inputs.absolutePosition = encoder.getAbsolutePosition();

        inputs.driveMotorSupplyCurrent = driveMotor.getSupplyCurrent();
        inputs.driveMotorStatorCurrent = driveMotor.getStatorCurrent();
        driveSupplyChargeUsedCoulomb.update(inputs.driveMotorSupplyCurrent);
        inputs.driveMotorSupplyCurrentOverTime = driveSupplyChargeUsedCoulomb.get();
        driveStatorChargeUsedCoulomb.update(inputs.driveMotorStatorCurrent);
        inputs.driveMotorStatorCurrentOverTime = driveStatorChargeUsedCoulomb.get();
        inputs.driveMotorPosition = driveMotor.getSelectedSensorPosition();
        inputs.driveMotorVelocity = getVelocity();
        inputs.driveMotorVelocitySetpoint = driveMotorVelocitySetpoint;

        inputs.angleMotorSupplyCurrent = angleMotor.getSupplyCurrent();
        inputs.angleMotorStatorCurrent = angleMotor.getStatorCurrent();
        angleSupplyChargeUsedCoulomb.update(inputs.angleMotorSupplyCurrent);
        inputs.angleMotorSupplyCurrentOverTime = angleSupplyChargeUsedCoulomb.get();
        angleStatorChargeUsedCoulomb.update(inputs.angleMotorStatorCurrent);
        inputs.angleMotorStatorCurrentOverTime = angleStatorChargeUsedCoulomb.get();
        inputs.angleMotorPosition = angleMotor.getSelectedSensorPosition();
        angleMotorPosition = inputs.angleMotorPosition;
        inputs.angleMotorVelocity = ticksPerMeter.toVelocity(angleMotor.getSelectedSensorVelocity());

        inputs.angle = getAngle();
        currentAngle = inputs.angle;

        inputs.angleSetpoint = angleSetpoint;

        inputs.moduleDistance = getModulePosition().distanceMeters;
    }

    @Override
    public double getAngle() {
        return AngleUtil.normalize(ticksPerRad.toUnits(angleMotor.getSelectedSensorPosition()));
    }

    @Override
    public void setAngle(double angle) {
        angleSetpoint = AngleUtil.normalize(angle);
        Rotation2d error = new Rotation2d(angle).minus(new Rotation2d(currentAngle));
        angleMotor.set(TalonFXControlMode.MotionMagic, angleMotorPosition + ticksPerRad.toTicks(error.getRadians()));
    }

    @Override
    public double getVelocity() {
        return ticksPerMeter.toVelocity(driveMotor.getSelectedSensorVelocity());
    }

    @Override
    public void setVelocity(double velocity) {
        var angleError = new Rotation2d(angleSetpoint).minus(new Rotation2d(currentAngle));
        velocity *= angleError.getCos();
        driveMotorVelocitySetpoint = velocity;
        driveMotor.set(TalonFXControlMode.Velocity, ticksPerMeter.toTicks100ms(velocity));
    }

    @Override
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(
                ticksPerMeter.toUnits(driveMotor.getSelectedSensorPosition()),
                new Rotation2d(getAngle())
        );
    }

    @Override
    public void updateOffset(double offset) {
        angleMotor.setSelectedSensorPosition(
                ((encoder.getAbsolutePosition() - offset) * 2048) / SwerveConstants.ANGLE_REDUCTION);
    }

    @Override
    public void neutralOutput() {
        driveMotor.neutralOutput();
        angleMotor.neutralOutput();
    }

    @Override
    public boolean encoderConnected() {
        return encoder.isConnected();
    }

    @Override
    public void checkModule() {
        driveMotor.set(TalonFXControlMode.PercentOutput, 0.8);
        angleMotor.set(TalonFXControlMode.PercentOutput, 0.2);
    }
}
