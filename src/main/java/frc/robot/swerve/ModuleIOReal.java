package frc.robot.swerve;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import utils.math.AngleUtil;
import utils.math.differential.Integral;

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
                        Slot0Configs drivePIDGains, Slot0Configs anglePIDGains,
                        MotionMagicConfigs motionMagicConfigs, int number) {
        this.driveMotor = new TalonFX(driveMotorID);
        this.angleMotor = new TalonFX(angleMotorID);

        this.encoder = new DutyCycleEncoder(encoderID);

        TalonFXConfiguration driveMotorConfig = new TalonFXConfiguration();
        driveMotorConfig.Slot0 = drivePIDGains;
        driveMotorConfig.Voltage.PeakForwardVoltage = SwerveConstants.VOLT_COMP_SATURATION;
        driveMotorConfig.Voltage.PeakReverseVoltage = -SwerveConstants.VOLT_COMP_SATURATION;
        driveMotorConfig.CurrentLimits.SupplyCurrentLimit = SwerveConstants.SUPPLY_CURRENT_LIMIT.currentLimit;
        driveMotorConfig.CurrentLimits.StatorCurrentLimit = SwerveConstants.STATOR_CURRENT_LIMIT.currentLimit;
        driveMotorConfig.ClosedLoopGeneral.ContinuousWrap = true;
        driveMotorConfig.Feedback.SensorToMechanismRatio = 1 / SwerveConstants.DRIVE_REDUCTION;
        driveMotorConfig.Feedback.RotorToSensorRatio = 1;
        driveMotor.getConfigurator().apply(driveMotorConfig);

        driveMotor.setNeutralMode(NeutralModeValue.Brake);
        driveMotor.setInverted(true);

        TalonFXConfiguration angleMotorConfig = new TalonFXConfiguration();
        angleMotorConfig.Slot0 = anglePIDGains;
        angleMotorConfig.MotionMagic = motionMagicConfigs;
        angleMotorConfig.Voltage.PeakForwardVoltage = SwerveConstants.VOLT_COMP_SATURATION;
        angleMotorConfig.Voltage.PeakReverseVoltage = -SwerveConstants.VOLT_COMP_SATURATION;
        angleMotorConfig.CurrentLimits.SupplyCurrentLimit = SwerveConstants.SUPPLY_CURRENT_LIMIT.currentLimit;
        angleMotorConfig.CurrentLimits.StatorCurrentLimit = SwerveConstants.STATOR_CURRENT_LIMIT.currentLimit;
        angleMotorConfig.ClosedLoopGeneral.ContinuousWrap = true;
        angleMotorConfig.Feedback.SensorToMechanismRatio = 1 / SwerveConstants.ANGLE_REDUCTION;
        angleMotorConfig.Feedback.RotorToSensorRatio = 1;
        angleMotorConfig.MotorOutput.DutyCycleNeutralDeadband = SwerveConstants.NEUTRAL_DEADBAND;
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
        return AngleUtil.normalize(Units.rotationsToRadians(angleMotor.getPosition().getValue()));
    }

    @Override
    public void setAngle(Rotation2d angle) {
        angleSetpoint = AngleUtil.normalize(angle.getRadians());
        Rotation2d error = angle.minus(new Rotation2d(currentAngle));
        angleMotor.setControl(
                new MotionMagicVoltage(angleMotor.getPosition().getValue() + error.getRotations())
                        .withEnableFOC(true));
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
        driveMotor.setControl(
                new VelocityVoltage(utils.units.Units.metersToRotations(
                        velocity,
                SwerveConstants.WHEEL_DIAMETER / 2
        )).withEnableFOC(true));
    }

    @Override
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(
                utils.units.Units.rpsToMetersPerSecond(
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
        driveMotor.setControl(new NeutralOut());
        angleMotor.setControl(new NeutralOut());
    }

    @Override
    public boolean encoderConnected() {
        return encoder.isConnected();
    }

    @Override
    public Command checkModule() {
        return new RunCommand(() -> {
            driveMotor.set(0.8);
            angleMotor.set(0.2);
        });
    }
}
