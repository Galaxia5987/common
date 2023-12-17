package frc.robot.swerve;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class SwerveModuleInputs implements LoggableInputs {
    public double driveMotorVelocity = 0;
    public double driveMotorVelocitySetpoint = 0;
    public double driveMotorSupplyCurrent = 0;
    public double driveMotorStatorCurrent = 0;
    public double driveMotorSupplyCurrentOverTime = 0;
    public double driveMotorStatorCurrentOverTime = 0;
    public double driveMotorPosition = 0;
    public double driveMotorAppliedVoltage = 0;

    public double angle = 0;
    public double angleSetpoint = 0;
    public double absolutePosition = 0;
    public double angleMotorVelocity = 0;
    public double angleMotorSupplyCurrent = 0;
    public double angleMotorStatorCurrent = 0;
    public double angleMotorSupplyCurrentOverTime = 0;
    public double angleMotorStatorCurrentOverTime = 0;
    public double angleMotorPosition = 0;
    public double angleMotorAppliedVoltage = 0;

    public double moduleDistance = 0;

    @Override
    public void toLog(LogTable table) {
        table.put("driveMotorVelocity", driveMotorVelocity);
        table.put("driveMotorVelocitySetpoint", driveMotorVelocitySetpoint);
        table.put("driveMotorSupplyCurrent", driveMotorSupplyCurrent);
        table.put("driveMotorStatorCurrent", driveMotorStatorCurrent);
        table.put("driveMotorSupplyCurrentOverTime", driveMotorSupplyCurrentOverTime);
        table.put("driveMotorStatorCurrentOverTime", driveMotorStatorCurrentOverTime);
        table.put("driveMotorPosition", driveMotorPosition);
        table.put("driveMotorAppliedVoltage", driveMotorAppliedVoltage);

        table.put("angle", angle);
        table.put("angleSetpoint", angleSetpoint);
        table.put("absolutePosition", absolutePosition);
        table.put("angleMotorVelocity", angleMotorVelocity);
        table.put("angleMotorSupplyCurrent", angleMotorSupplyCurrent);
        table.put("angleMotorStatorCurrent", angleMotorStatorCurrent);
        table.put("angleMotorSupplyCurrentOverTime", angleMotorSupplyCurrentOverTime);
        table.put("angleMotorStatorCurrentOverTime", angleMotorStatorCurrentOverTime);
        table.put("angleMotorPosition", angleMotorPosition);
        table.put("angleMotorAppliedVoltage", angleMotorAppliedVoltage);

        table.put("moduleDistance", moduleDistance);
    }

    @Override
    public void fromLog(LogTable table) {
        table.get("driveMotorVelocity", driveMotorVelocity);
        table.get("driveMotorVelocitySetpoint", driveMotorVelocitySetpoint);
        table.get("driveMotorSupplyCurrent", driveMotorSupplyCurrent);
        table.get("driveMotorStatorCurrent", driveMotorStatorCurrent);
        table.get("driveMotorSupplyCurrentOverTime", driveMotorSupplyCurrentOverTime);
        table.get("driveMotorStatorCurrentOverTime", driveMotorStatorCurrentOverTime);
        table.get("driveMotorPosition", driveMotorPosition);
        table.get("driveMotorAppliedVoltage", driveMotorAppliedVoltage);

        table.get("angle", angle);
        table.get("angleSetpoint", angleSetpoint);
        table.get("absolutePosition", absolutePosition);
        table.get("angleMotorVelocity", angleMotorVelocity);
        table.get("angleMotorSupplyCurrent", angleMotorSupplyCurrent);
        table.get("angleMotorStatorCurrent", angleMotorStatorCurrent);
        table.get("angleMotorSupplyCurrentOverTime", angleMotorSupplyCurrentOverTime);
        table.get("angleMotorStatorCurrentOverTime", angleMotorStatorCurrentOverTime);
        table.get("angleMotorPosition", angleMotorPosition);
        table.get("angleMotorAppliedVoltage", angleMotorAppliedVoltage);

        table.get("moduleDistance", moduleDistance);
    }
}
