package frc.robot.swerve;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import lib.math.AngleUtil;
import lib.math.differential.Integral;
import lib.units.Units;

public class ModuleIOReal implements ModuleIO {

    private final TalonFX driveMotor;
    private final TalonFX angleMotor;
    private final DutyCycleEncoder encoder;

    private final Integral driveSupplyChargeUsedCoulomb = new Integral();
    private final Integral driveStatorChargeUsedCoulomb = new Integral();
    private final Integral angleSupplyChargeUsedCoulomb = new Integral();
    private final Integral angleStatorChargeUsedCoulomb = new Integral();
    private final MotionMagicVoltage angleControlRequest =
            new MotionMagicVoltage(0).withEnableFOC(true).withSlot(0);
    private final VelocityVoltage velocityControlRequest =
            new VelocityVoltage(0).withEnableFOC(true);
    private Rotation2d angleSetpoint = new Rotation2d();
    private Rotation2d currentAngle = new Rotation2d();
    private double driveMotorVelocitySetpoint = 0;

    public ModuleIOReal(
            int driveMotorID,
            int angleMotorID,
            int encoderID,
            TalonFXConfiguration driveConfig,
            TalonFXConfiguration angleConfig) {

        this.driveMotor = new TalonFX(driveMotorID);
        this.angleMotor = new TalonFX(angleMotorID);

        this.encoder = new DutyCycleEncoder(encoderID);

        driveConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        driveMotor.getConfigurator().apply(driveConfig);
        driveMotor.setPosition(0);

        angleConfig.ClosedLoopGeneral.ContinuousWrap = true;
        angleConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        angleMotor.getConfigurator().apply(angleConfig);
        angleMotor.setPosition(0);

        driveMotor.setNeutralMode(NeutralModeValue.Brake);
        angleMotor.setNeutralMode(NeutralModeValue.Brake);
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
        currentAngle = inputs.angle.getRadians();

        inputs.angleSetpoint = Rotation2d.fromRadians(angleSetpoint);

        inputs.moduleDistance = getModulePosition().distanceMeters;
        inputs.moduleState = getModuleState();
    }

    @Override
    public Rotation2d getAngle() {
        return AngleUtil.normalize(
                Rotation2d.fromRadians(
                        ticksPerRad.toUnits(angleMotor.getSelectedSensorPosition())));
    }

    @Override
    public void setAngle(Rotation2d angle) {
        angleSetpoint = AngleUtil.normalize(angle.getRadians());
        Rotation2d error = angle.minus(new Rotation2d(currentAngle));
        angleMotor.set(
                TalonFXControlMode.MotionMagic,
                angleMotorPosition + ticksPerRad.toTicks(error.getRadians()));
    }

    @Override
    public double getVelocity() {
        return Units.rpsToMetersPerSecond(
                driveMotor.getVelocity().getValue(), SwerveConstants.WHEEL_DIAMETER / 2);
    }

    @Override
    public void setVelocity(double velocity) {
        var angleError = angleSetpoint.minus(currentAngle);
        velocity *= angleError.getCos();

        driveMotorVelocitySetpoint = velocity;

        velocityControlRequest
                .withVelocity(
                        1
                                / Units.rpsToMetersPerSecond(
                                        velocity, SwerveConstants.WHEEL_DIAMETER / 2))
                .withEnableFOC(true);
        driveMotor.setControl(velocityControlRequest);
    }

    @Override
    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(getVelocity(), getAngle());
    }

    @Override
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(
                ticksPerMeter.toUnits(driveMotor.getSelectedSensorPosition()), getAngle());
    }

    @Override
    public void updateOffset(double offset) {
        angleMotor.setPosition(encoder.getAbsolutePosition() - offset);
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
    public Command checkModule() {
        return Commands.run(
                () -> {
                    driveMotor.set(TalonFXControlMode.PercentOutput, 0.8);
                    angleMotor.set(TalonFXControlMode.PercentOutput, 0.2);
                });
    }
}
