package swerve;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import utils.math.AngleUtil;
import utils.math.differential.Integral;
import utils.units.Units;

public class ModuleIOReal implements ModuleIO {

    private final TalonFX driveMotor;
    private final TalonFX angleMotor;

    private final DutyCycleEncoder encoder;

    private final Integral driveSupplyChargeUsedCoulomb = new Integral();
    private final Integral driveStatorChargeUsedCoulomb = new Integral();
    private final Integral angleSupplyChargeUsedCoulomb = new Integral();
    private final Integral angleStatorChargeUsedCoulomb = new Integral();
    private double angleSetpoint;
    private double currentAngle;
    private double driveMotorVelocitySetpoint;

    public ModuleIOReal(int driveMotorID, int angleMotorID, int encoderID,
                        double[] motionMagicConfigs, int number) {
        this.driveMotor = new TalonFX(driveMotorID);
        this.angleMotor = new TalonFX(angleMotorID);

        this.encoder = new DutyCycleEncoder(encoderID);

        TalonFXConfiguration driveMotorConfig = new TalonFXConfiguration();
        Slot0Configs PIDGains = new Slot0Configs()
                .withKP(SwerveConstants.DRIVE_kP)
                .withKI(SwerveConstants.DRIVE_kI)
                .withKD(SwerveConstants.DRIVE_kD)
                .withKV(SwerveConstants.DRIVE_KF);

        driveMotorConfig.Slot0 = PIDGains;
        driveMotorConfig.Voltage.PeakForwardVoltage = SwerveConstants.VOLT_COMP_SATURATION;
        driveMotorConfig.Voltage.PeakReverseVoltage = -SwerveConstants.VOLT_COMP_SATURATION;
        driveMotorConfig.CurrentLimits.SupplyCurrentLimit = SwerveConstants.SUPPLY_CURRENT_LIMIT.currentLimit;
        driveMotorConfig.CurrentLimits.StatorCurrentLimit = SwerveConstants.STATOR_CURRENT_LIMIT.currentLimit;
        driveMotorConfig.Feedback.SensorToMechanismRatio = 1 / SwerveConstants.DRIVE_REDUCTION;
        driveMotorConfig.Feedback.RotorToSensorRatio = 1;
        driveMotor.getConfigurator().apply(driveMotorConfig);

        driveMotor.setNeutralMode(NeutralModeValue.Brake);
        driveMotor.setInverted(true);

        TalonFXConfiguration angleMotorConfig = new TalonFXConfiguration();
        angleMotorConfig.Voltage.PeakForwardVoltage = SwerveConstants.VOLT_COMP_SATURATION;
        angleMotorConfig.Voltage.PeakReverseVoltage = -SwerveConstants.VOLT_COMP_SATURATION;
        angleMotorConfig.CurrentLimits.SupplyCurrentLimit = SwerveConstants.SUPPLY_CURRENT_LIMIT.currentLimit;
        angleMotorConfig.CurrentLimits.StatorCurrentLimit = SwerveConstants.STATOR_CURRENT_LIMIT.currentLimit;
        angleMotorConfig.Feedback.SensorToMechanismRatio = 1 / SwerveConstants.ANGLE_REDUCTION;
        angleMotorConfig.Feedback.RotorToSensorRatio = 1;
        angleMotorConfig.MotorOutput.DutyCycleNeutralDeadband = SwerveConstants.NEUTRAL_DEADBAND;
        angleMotorConfig.Slot0.kP = motionMagicConfigs[0];
        angleMotorConfig.Slot0.kI = motionMagicConfigs[1];
        angleMotorConfig.Slot0.kD = motionMagicConfigs[2];
        angleMotorConfig.Slot0.kV = motionMagicConfigs[3];
        angleMotorConfig.Slot0.kS = motionMagicConfigs[4];
        angleMotorConfig.MotionMagic.MotionMagicJerk = motionMagicConfigs[5];
        angleMotorConfig.MotionMagic.MotionMagicCruiseVelocity = motionMagicConfigs[6];
        angleMotorConfig.MotionMagic.MotionMagicAcceleration = motionMagicConfigs[7];
        angleMotor.getConfigurator().apply(angleMotorConfig);

        angleMotor.setNeutralMode(NeutralModeValue.Brake);
        angleMotor.setInverted(true);
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
        inputs.angleMotorVelocity = angleMotor.getVelocity().getValue();

        inputs.angle = getAngle();
        currentAngle = inputs.angle;

        inputs.angleSetpoint = angleSetpoint;

        inputs.moduleDistance = getModulePosition().distanceMeters;
    }

    @Override
    public double getAngle() {
        return AngleUtil.normalize(Units.rotationsToRad(angleMotor.getPosition().getValue()));
    }

    @Override
    public void setAngle(double angle) {
        angleSetpoint = AngleUtil.normalize(angle);
        Rotation2d error = new Rotation2d(angle).minus(new Rotation2d(currentAngle));
        var motorRequest = new MotionMagicVoltage(angleMotor.getPosition().getValue() + error.getRotations());
        angleMotor.setControl(motorRequest);
    }

    @Override
    public double getVelocity() {
        return driveMotor.getVelocity().getValue();
    }

    @Override
    public void setVelocity(double velocity) {
        var angleError = new Rotation2d(angleSetpoint).minus(new Rotation2d(currentAngle));
        velocity *= angleError.getCos();

        driveMotorVelocitySetpoint = velocity;
        var motorRequest = new VelocityDutyCycle(
                Units.metersPerSecondToRps(
                        velocity,
                        SwerveConstants.WHEEL_DIAMETER / 2
                )
        );
        driveMotor.setControl(motorRequest);
    }

    @Override
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(
                Units.rpsToMetersPerSecond(
                        driveMotor.getPosition().getValue(),
                        SwerveConstants.WHEEL_DIAMETER / 2
                ),
                new Rotation2d(getAngle())
        );
    }

    @Override
    public void updateOffset(double offset) {
        angleMotor.setPosition(
                ((encoder.getAbsolutePosition() - offset) * 2048) / SwerveConstants.ANGLE_REDUCTION);
    }

    @Override
    public void neutralOutput() {
        var motorRequest = new NeutralOut();
        driveMotor.setControl(motorRequest);
        angleMotor.setControl(motorRequest);
    }

    @Override
    public boolean encoderConnected() {
        return encoder.isConnected();
    }

    @Override
    public void checkModule() {
        var driveMotorRequest = new DutyCycleOut(0.8);
        var angleMotorRequest = new DutyCycleOut(0.2);
        driveMotor.setControl(driveMotorRequest);
        angleMotor.setControl(angleMotorRequest);
    }
}
