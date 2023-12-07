package utils.motors;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;

public class TalonFXReal implements TalonFXMotor {

    private final TalonFX motor;

    private double rotorVelocity;
    private double rotorPosition;
    private double rotorAcceleration;
    private double appliedCurrent;
    private double appliedVoltage;

    public TalonFXReal(int id, String canbus) {
        motor = new TalonFX(id, canbus);
    }

    public TalonFXReal(int id) {
        motor = new TalonFX(id);
    }

    @Override
    public void update(double timestampSeconds) {
        rotorVelocity = motor.getRotorVelocity().getValue();
        rotorPosition = motor.getRotorPosition().getValue();
        rotorAcceleration = motor.getAcceleration().getValue();
        appliedCurrent = motor.getStatorCurrent().getValue();
        appliedVoltage = motor.getMotorVoltage().getValue();
    }

    @Override
    public StatusCode configure(TalonFXConfiguration config) {
        return motor.getConfigurator().apply(config);
    }

    @Override
    public StatusCode setControl(DutyCycleOut request) {
        return motor.setControl(request);
    }

    @Override
    public StatusCode setControl(VoltageOut request) {
        return motor.setControl(request);
    }

    @Override
    public StatusCode setControl(PositionDutyCycle request) {
        return motor.setControl(request);
    }

    @Override
    public StatusCode setControl(PositionVoltage request) {
        return motor.setControl(request);
    }

    @Override
    public StatusCode setControl(PositionTorqueCurrentFOC request) {
        return motor.setControl(request);
    }

    @Override
    public StatusCode setControl(VelocityDutyCycle request) {
        return motor.setControl(request);
    }

    @Override
    public StatusCode setControl(VelocityVoltage request) {
        return motor.setControl(request);
    }

    @Override
    public StatusCode setControl(VelocityTorqueCurrentFOC request) {
        return motor.setControl(request);
    }

    @Override
    public StatusCode setControl(MotionMagicDutyCycle request) {
        return motor.setControl(request);
    }

    @Override
    public StatusCode setControl(MotionMagicVoltage request) {
        return motor.setControl(request);
    }

    @Override
    public StatusCode setControl(MotionMagicTorqueCurrentFOC request) {
        return motor.setControl(request);
    }

    @Override
    public double getRotorVelocity() {
        return rotorVelocity;
    }

    @Override
    public double getRotorPosition() {
        return rotorPosition;
    }

    @Override
    public double getAcceleration() {
        return rotorAcceleration;
    }

    @Override
    public double getAppliedCurrent() {
        return appliedCurrent;
    }

    @Override
    public double getAppliedVoltage() {
        return appliedVoltage;
    }
}
