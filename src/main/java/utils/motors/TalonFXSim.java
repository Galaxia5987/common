package utils.motors;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import utils.math.differential.Derivative;
import utils.units.Units;

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
        acceleration.update(getRotorVelocity(), timestampSeconds);

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
        return this.setControl(new VoltageOut(voltageRequest));
    }

    public StatusCode setControl(PositionVoltage request) {
        return this.setControl(new PositionDutyCycle(request.Position));
    }

    public StatusCode setControl(PositionTorqueCurrentFOC request) {
        return this.setControl(new PositionDutyCycle(request.Position));
    }

    public StatusCode setControl(VelocityDutyCycle request) {
        voltageRequest = controller.calculate(
                Units.rpmToRps(motorSim.getAngularVelocityRPM()), request.Velocity);
        return this.setControl(new VoltageOut(voltageRequest));
    }

    public StatusCode setControl(VelocityVoltage request) {
        return this.setControl(new VelocityDutyCycle(request.Velocity));
    }

    public StatusCode setControl(VelocityTorqueCurrentFOC request) {
        return this.setControl(new VelocityDutyCycle(request.Velocity));
    }

    public StatusCode setControl(MotionMagicDutyCycle request) {
        voltageRequest = profiledController.calculate(
                motorSim.getAngularPositionRotations(), request.Position);
        return this.setControl(new VelocityDutyCycle(voltageRequest));
    }

    public StatusCode setControl(MotionMagicVoltage request) {
        return this.setControl(new MotionMagicDutyCycle(request.Position));
    }

    public StatusCode setControl(MotionMagicTorqueCurrentFOC request) {
        return this.setControl(new MotionMagicDutyCycle(request.Position));
    }

    public double getRotorVelocity() {
        return Units.rpmToRps(motorSim.getAngularVelocityRPM());
    }

    public double getRotorPosition() {
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