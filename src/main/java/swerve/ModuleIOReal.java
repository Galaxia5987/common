package swerve;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants;
import frc.robot.Ports;
import utils.math.AngleUtil;
import utils.math.differential.Integral;
import utils.units.UnitModel;

public class
ModuleIOReal implements ModuleIO { //TODO: keep updating to phoenix6 and FIX UNITS!!

    private final TalonFX driveMotor;
    private final TalonFXConfiguration driveMotorConfig = new TalonFXConfiguration();
    private final TalonFXConfigurator driveMotorConfigurator;

    private final TalonFX angleMotor;
    private final TalonFXConfiguration angleMotorConfig = new TalonFXConfiguration();
    private final TalonFXConfigurator angleMotorConfigurator;

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

        driveMotorConfigurator = driveMotor.getConfigurator();
        angleMotorConfigurator = angleMotor.getConfigurator();

        driveMotorConfigurator.apply(new TalonFXConfiguration()); //factory default
        angleMotorConfigurator.apply(new TalonFXConfiguration());

        driveMotorConfig.Slot0.kP = SwerveConstants.DRIVE_kP;
        driveMotorConfig.Slot0.kI = SwerveConstants.DRIVE_kI;
        driveMotorConfig.Slot0.kD = SwerveConstants.DRIVE_kD;
        driveMotorConfig.Slot0.kV = SwerveConstants.DRIVE_KF;
        driveMotorConfig.Voltage.PeakForwardVoltage = SwerveConstants.VOLT_COMP_SATURATION;
        driveMotorConfig.Voltage.PeakReverseVoltage = -SwerveConstants.VOLT_COMP_SATURATION;
        driveMotor.setNeutralMode(NeutralModeValue.Brake);
        driveMotorConfig.CurrentLimits.SupplyCurrentLimit = SwerveConstants.SUPPLY_CURRENT_LIMIT.currentLimit;
        driveMotorConfig.CurrentLimits.StatorCurrentLimit = SwerveConstants.STATOR_CURRENT_LIMIT.currentLimit;
        driveMotor.setInverted(true);
        driveMotorConfigurator.apply(driveMotorConfig);

        angleMotorConfig.Voltage.PeakForwardVoltage = SwerveConstants.VOLT_COMP_SATURATION;
        angleMotorConfig.Voltage.PeakReverseVoltage = -SwerveConstants.VOLT_COMP_SATURATION;
        angleMotor.setNeutralMode(NeutralModeValue.Brake);
        angleMotorConfig.CurrentLimits.SupplyCurrentLimit = SwerveConstants.SUPPLY_CURRENT_LIMIT.currentLimit;
        angleMotorConfig.CurrentLimits.StatorCurrentLimit = SwerveConstants.STATOR_CURRENT_LIMIT.currentLimit;
        angleMotor.setInverted(true);
        angleMotorConfig.MotorOutput.DutyCycleNeutralDeadband = SwerveConstants.NEUTRAL_DEADBAND;

        angleMotorConfig.Slot0.kP = motionMagicConfigs[0];
        angleMotorConfig.Slot0.kI = motionMagicConfigs[1];
        angleMotorConfig.Slot0.kD = motionMagicConfigs[2];
        angleMotorConfig.Slot0.kV = motionMagicConfigs[3];
        angleMotorConfig.Slot0.kS = motionMagicConfigs[4];
        angleMotorConfig.MotionMagic.MotionMagicJerk = motionMagicConfigs[5];
        angleMotorConfig.MotionMagic.MotionMagicCruiseVelocity = motionMagicConfigs[6];
        angleMotorConfig.MotionMagic.MotionMagicAcceleration = motionMagicConfigs[7];
        angleMotorConfigurator.apply(driveMotorConfig);
    }

    @Override
    public void updateInputs(SwerveModuleInputs inputs) {
        inputs.absolutePosition = encoder.getAbsolutePosition();

        inputs.driveMotorSupplyCurrent = driveMotor.getSupplyCurrent().getValue();
        inputs.driveMotorStatorCurrent = driveMotor.getStatorCurrent().getValue();
        driveSupplyChargeUsedCoulomb.update(inputs.driveMotorSupplyCurrent);
        inputs.driveMotorSupplyCurrentOverTime = driveSupplyChargeUsedCoulomb.get();
        driveStatorChargeUsedCoulomb.update(inputs.driveMotorStatorCurrent);
        inputs.driveMotorStatorCurrentOverTime = driveStatorChargeUsedCoulomb.get();
        inputs.driveMotorPosition = driveMotor.getRotorPosition().getValue();
        inputs.driveMotorVelocity = getVelocity();
        inputs.driveMotorVelocitySetpoint = driveMotorVelocitySetpoint;

        inputs.angleMotorSupplyCurrent = angleMotor.getSupplyCurrent().getValue();
        inputs.angleMotorStatorCurrent = angleMotor.getStatorCurrent().getValue();
        angleSupplyChargeUsedCoulomb.update(inputs.angleMotorSupplyCurrent);
        inputs.angleMotorSupplyCurrentOverTime = angleSupplyChargeUsedCoulomb.get();
        angleStatorChargeUsedCoulomb.update(inputs.angleMotorStatorCurrent);
        inputs.angleMotorStatorCurrentOverTime = angleStatorChargeUsedCoulomb.get();
        inputs.angleMotorPosition = angleMotor.getRotorPosition().getValue();
        angleMotorPosition = inputs.angleMotorPosition;
        inputs.angleMotorVelocity = ticksPerMeter.toVelocity(angleMotor.getVelocity().getValue());

        inputs.angle = getAngle();
        currentAngle = inputs.angle;

        inputs.angleSetpoint = angleSetpoint;

        inputs.moduleDistance = getModulePosition().distanceMeters;
    }

    @Override
    public double getAngle() {
        return AngleUtil.normalize(ticksPerRad.toUnits(angleMotor.getRotorPosition().getValue()));
    }

    @Override
    public void setAngle(double angle) {
        angleSetpoint = AngleUtil.normalize(angle);
        Rotation2d error = new Rotation2d(angle).minus(new Rotation2d(currentAngle));
        var motorRequest = new MotionMagicVoltage(angleMotorPosition + ticksPerRad.toTicks(error.getRadians()));
        angleMotor.setControl(motorRequest);
    }

    @Override
    public double getVelocity() {
        return ticksPerMeter.toVelocity(driveMotor.getRotorVelocity().getValue());
    }

    @Override
    public void setVelocity(double velocity) {
        var angleError = new Rotation2d(angleSetpoint).minus(new Rotation2d(currentAngle));
        velocity *= angleError.getCos();
        driveMotorVelocitySetpoint = velocity;
        var motorRequest = new VelocityDutyCycle(ticksPerMeter.toTicks100ms(velocity));
        driveMotor.setControl(motorRequest);
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
