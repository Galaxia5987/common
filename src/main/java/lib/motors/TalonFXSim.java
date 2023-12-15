package lib.motors;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import lib.math.differential.Derivative;
import lib.units.Units;

public class TalonFXSim {

    private final DCMotorSim motorSim;

    private PIDController controller = null;
    private ProfiledPIDController profiledController = null;

    private final Derivative acceleration = new Derivative();
    private double lastTimestampSeconds = 0;

    private double voltageRequest;

    public TalonFXSim(int numMotors, double gearing, double jKgMetersSquared) {
        DCMotor motor = DCMotor.getFalcon500(numMotors);

        motorSim = new DCMotorSim(motor, gearing, jKgMetersSquared);
    }

    public void update(double timestampSeconds) {
        motorSim.update(timestampSeconds - lastTimestampSeconds);
        acceleration.update(getVelocity(), timestampSeconds);

        lastTimestampSeconds = timestampSeconds;
    }

    public TalonFXSim setController(PIDController controller) {
        this.controller = controller;
        return this;
    }

    public TalonFXSim setProfiledController(ProfiledPIDController profiledController) {
        this.profiledController = profiledController;
        return this;
    }

    public StatusCode setControl(DutyCycleOut request) {
        return this.setControl(new VoltageOut(request.Output * 12));
    }

    public StatusCode setControl(VoltageOut request) {
        voltageRequest = request.Output;
        motorSim.setInputVoltage(voltageRequest);
        return StatusCode.OK;
    }

    public StatusCode setControl(PositionDutyCycle request) {
        voltageRequest = controller.calculate(
                motorSim.getAngularPositionRotations(), request.Position);
        return this.setControl(new VoltageOut(voltageRequest + 12 * request.FeedForward));
    }

    public StatusCode setControl(PositionVoltage request) {
        voltageRequest = controller.calculate(
                motorSim.getAngularPositionRotations(), request.Position);
        return this.setControl(new VoltageOut(voltageRequest + request.FeedForward));
    }

    public StatusCode setControl(VelocityDutyCycle request) {
        voltageRequest = controller.calculate(
                Units.rpmToRps(motorSim.getAngularVelocityRPM()), request.Velocity);
        return this.setControl(new VoltageOut(voltageRequest + 12 * request.FeedForward));
    }

    public StatusCode setControl(VelocityVoltage request) {
        voltageRequest = controller.calculate(
                Units.rpmToRps(motorSim.getAngularVelocityRPM()), request.Velocity);
        return this.setControl(new VoltageOut(voltageRequest + request.FeedForward));
    }

    public StatusCode setControl(MotionMagicDutyCycle request) {
        voltageRequest = profiledController.calculate(
                motorSim.getAngularPositionRotations(), request.Position);
        return this.setControl(new VoltageOut(voltageRequest + 12 * request.FeedForward));
    }

    public StatusCode setControl(MotionMagicVoltage request) {
        voltageRequest = profiledController.calculate(
                motorSim.getAngularPositionRotations(), request.Position);
        return this.setControl(new VoltageOut(voltageRequest + request.FeedForward));
    }

    public double getVelocity() {
        return Units.rpmToRps(motorSim.getAngularVelocityRPM());
    }

    public double getPosition() {
        return motorSim.getAngularPositionRotations();
    }

    public double getAcceleration() {
        return acceleration.get();
    }

    public double getAppliedCurrent() {
        return motorSim.getCurrentDrawAmps();
    }

    public double getAppliedVoltage() {
        return voltageRequest;
    }
}