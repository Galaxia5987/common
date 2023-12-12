package utils.motors;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;

public interface TalonFXMotor {

    void update(double timestampSeconds);

    StatusCode configure(TalonFXConfiguration config);

    StatusCode setControl(DutyCycleOut request);

    StatusCode setControl(VoltageOut request);

    StatusCode setControl(PositionDutyCycle request);

    StatusCode setControl(PositionVoltage request);

    StatusCode setControl(PositionTorqueCurrentFOC request);

    StatusCode setControl(VelocityDutyCycle request);

    StatusCode setControl(VelocityVoltage request);

    StatusCode setControl(VelocityTorqueCurrentFOC request);

    StatusCode setControl(MotionMagicDutyCycle request);

    StatusCode setControl(MotionMagicVoltage request);

    StatusCode setControl(MotionMagicTorqueCurrentFOC request);

    double getVelocity(double rotorToMechanismRatio);

    double getRotorVelocity();

    double getPosition(double rotorToMechanismRatio);

    double getRotorPosition();

    double getAcceleration();

    double getAppliedCurrent();

    double getAppliedVoltage();
}
