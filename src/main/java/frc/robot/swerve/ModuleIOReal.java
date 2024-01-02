package frc.robot.swerve;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants;
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
    private Rotation2d angleSetpoint;
    private Rotation2d currentAngle;
    private double driveMotorVelocitySetpoint;

    private MotionMagicVoltage angleControlRequest = new MotionMagicVoltage(0).withEnableFOC(true);
    private VelocityVoltage velocityControlRequest = new VelocityVoltage(0).withEnableFOC(true);

    public ModuleIOReal(int driveMotorID, int angleMotorID, int encoderID, int number,
                        TalonFXConfiguration driveConfig, TalonFXConfiguration angleConfig) {
        this.driveMotor = new TalonFX(driveMotorID);
        this.angleMotor = new TalonFX(angleMotorID);

        this.encoder = new DutyCycleEncoder(encoderID);

        driveConfig.ClosedLoopGeneral.ContinuousWrap = true;
        driveMotor.getConfigurator().apply(driveConfig);

        angleConfig.ClosedLoopGeneral.ContinuousWrap = true;
        angleMotor.getConfigurator().apply(angleConfig);

        driveMotor.setNeutralMode(NeutralModeValue.Brake);
        driveMotor.setInverted(true);
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
        inputs.moduleState = getModuleState();
    }

    @Override
    public Rotation2d getAngle() {
        return new Rotation2d(AngleUtil.normalize(Units.rotationsToRadians(angleMotor.getPosition().getValue())));
    }

    @Override
    public void setAngle(Rotation2d angle) {
        angleSetpoint = angle;
        Rotation2d error = angle.minus(currentAngle);
        angleControlRequest.withPosition(angleMotor.getPosition().getValue() + error.getRotations());
        angleMotor.setControl(angleControlRequest);
    }

    @Override
    public double getVelocity() {
        return driveMotor.getVelocity().getValue();
    }

    @Override
    public void setVelocity(double velocity) {
        var angleError = angleSetpoint.minus(currentAngle);
        velocity *= angleError.getCos();

        driveMotorVelocitySetpoint = velocity;

        velocityControlRequest.withVelocity(utils.units.Units.metersToRotations(
                        velocity,
                        SwerveConstants.WHEEL_DIAMETER / 2));
        driveMotor.setControl(velocityControlRequest);
    }

    @Override
    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(
                utils.units.Units.rpsToMetersPerSecond(
                        getVelocity(), SwerveConstants.WHEEL_DIAMETER/2)
                , getAngle()
        );
    }

    @Override
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(
                utils.units.Units.rpsToMetersPerSecond(
                        driveMotor.getPosition().getValue(),
                        SwerveConstants.WHEEL_DIAMETER / 2
                ),
                getAngle()
        );
    }

    @Override
    public void updateOffset(double offset) {
        angleMotor.setPosition(
                ((encoder.getAbsolutePosition() - offset) * Constants.FALCON_TICKS) / SwerveConstants.ANGLE_REDUCTION);
    }

    @Override
    public void stopMotor() {
        driveMotor.stopMotor();
        angleMotor.stopMotor();
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
