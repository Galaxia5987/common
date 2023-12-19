package lib.motors;

import com.ctre.phoenix6.controls.*;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import lib.math.differential.Derivative;
import lib.units.Units;

public class TalonFXSim extends SimMotor {

    private final Derivative acceleration = new Derivative();

    public TalonFXSim(LinearSystem<N2, N1, N2> model, int numMotors, double gearing) {
        super(model, DCMotor.getFalcon500(numMotors), gearing);
    }

    public TalonFXSim(int numMotors, double gearing, double jKgMetersSquared) {
        super(DCMotor.getFalcon500(numMotors), jKgMetersSquared, gearing);
    }

    public void setControl(DutyCycleOut request) {
        this.setControl(new VoltageOut(request.Output * 12));
    }

    public void setControl(VoltageOut request) {
        voltageRequest = request.Output;
        motorSim.setInputVoltage(voltageRequest);
    }

    public void setControl(PositionDutyCycle request) {
        voltageRequest = controller.calculate(
                motorSim.getAngularPositionRotations(), request.Position);
        this.setControl(new VoltageOut(voltageRequest + 12 * request.FeedForward));
    }

    public void setControl(PositionVoltage request) {
        voltageRequest = controller.calculate(
                motorSim.getAngularPositionRotations(), request.Position);
        this.setControl(new VoltageOut(voltageRequest + request.FeedForward));
    }

    public void setControl(VelocityDutyCycle request) {
        voltageRequest = controller.calculate(
                Units.rpmToRps(motorSim.getAngularVelocityRPM()), request.Velocity);
        this.setControl(new VoltageOut(voltageRequest + 12 * request.FeedForward));
    }

    public void setControl(VelocityVoltage request) {
        voltageRequest = controller.calculate(
                Units.rpmToRps(motorSim.getAngularVelocityRPM()), request.Velocity);
        this.setControl(new VoltageOut(voltageRequest + request.FeedForward));
    }

    public void setControl(MotionMagicDutyCycle request) {
        voltageRequest = profiledController.calculate(
                motorSim.getAngularPositionRotations(), request.Position);
        this.setControl(new VoltageOut(voltageRequest + 12 * request.FeedForward));
    }

    public void setControl(MotionMagicVoltage request) {
        voltageRequest = profiledController.calculate(
                motorSim.getAngularPositionRotations(), request.Position);
        this.setControl(new VoltageOut(voltageRequest + request.FeedForward));
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