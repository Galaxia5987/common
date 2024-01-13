package frc.robot.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import lib.PhoenixOdometryThread;
import lib.math.AngleUtil;
import lib.math.differential.Integral;
import lib.units.Units;

import java.util.ArrayList;
import java.util.List;
import java.util.Queue;

public class ModuleIOReal implements ModuleIO {

    private final TalonFX driveMotor;
    private final TalonFX angleMotor;
    private final DutyCycleEncoder encoder;

    private final Integral driveSupplyChargeUsedCoulomb = new Integral();
    private final Integral driveStatorChargeUsedCoulomb = new Integral();
    private final Integral angleSupplyChargeUsedCoulomb = new Integral();
    private final Integral angleStatorChargeUsedCoulomb = new Integral();
    private final PositionVoltage angleControlRequest =
            new PositionVoltage(0).withEnableFOC(true).withSlot(0);
    private final VelocityVoltage velocityControlRequest =
            new VelocityVoltage(0).withEnableFOC(true);
    private Rotation2d angleSetpoint = new Rotation2d();
    private Rotation2d currentAngle = new Rotation2d();
    private double driveMotorVelocitySetpoint = 0;
    private final Queue<Double> distanceQueue;
    private final Queue<Double> angleQueue;

    public ModuleIOReal(
            int driveMotorID,
            int angleMotorID,
            int encoderID,
            TalonFXConfiguration driveConfig,
            TalonFXConfiguration angleConfig) {

        this.driveMotor = new TalonFX(driveMotorID, "swerveDrive");
        this.angleMotor = new TalonFX(angleMotorID, "swerveDrive");

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

        var drivePositionSignal = driveMotor.getPosition();
        var driveVelocitySignal = driveMotor.getVelocity();
        distanceQueue = PhoenixOdometryThread
                .getInstance()
                .registerSignal(driveMotor, drivePositionSignal, driveVelocitySignal);

        var anglePositionSignal = angleMotor.getPosition();
        var angleVelocitySignal = angleMotor.getVelocity();
        angleQueue = PhoenixOdometryThread
                .getInstance()
                .registerSignal(angleMotor, anglePositionSignal, angleVelocitySignal);

        BaseStatusSignal.setUpdateFrequencyForAll(
                SwerveConstants.ODOMETRY_FREQUENCY,
                drivePositionSignal, anglePositionSignal,
                driveVelocitySignal, angleVelocitySignal);
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

        List<Double> distanceList = distanceQueue.stream().toList();
        int nD = distanceList.size();
        inputs.highFreqDistances = new double[nD];
        for (int i = 0; i < nD; i++) {
            inputs.highFreqDistances[i] = Units.rpsToMetersPerSecond(
                    distanceList.get(i), SwerveConstants.WHEEL_DIAMETER/2
            );
        }
        distanceQueue.clear();

        List<Double> angleList = angleQueue.stream().toList();
        int nA = angleList.size();
        inputs.highFreqAngles = new double[nA];
        for (int i = 0; i < nA; i++) {
            inputs.highFreqAngles[i] = Units.rpsToRadsPerSec(angleList.get(i));
        }
        angleQueue.clear();
    }

    @Override
    public Rotation2d getAngle() {
        return AngleUtil.normalize(Rotation2d.fromRotations(angleMotor.getPosition().getValue()));
    }

    @Override
    public void setAngle(Rotation2d angle) {
        angle = AngleUtil.normalize(angle);
        angleSetpoint = angle;
        Rotation2d error = angle.minus(currentAngle);
        angleControlRequest
                .withPosition(angleMotor.getPosition().getValue() + error.getRotations())
                .withFeedForward(SwerveConstants.kF)
                .withEnableFOC(true);
        angleMotor.setControl(angleControlRequest);
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
                        Units.metersToRotations(
                                velocity, SwerveConstants.WHEEL_DIAMETER / 2)
                )
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
                Units.rpsToMetersPerSecond(
                        driveMotor.getPosition().getValue(), SwerveConstants.WHEEL_DIAMETER / 2),
                getAngle());
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
    public void checkModule() {
        driveMotor.set(0.8);
        angleMotor.set(0.2);
    }
}
