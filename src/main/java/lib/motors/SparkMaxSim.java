package lib.motors;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import lib.math.differential.Derivative;

public class SparkMaxSim {

    private final DCMotorSim motorSim;
    private PIDController controller = null;
    private ProfiledPIDController profiledController = null;
    private double lastTimestampSeconds = 0;

    private double voltageRequest;

    public SparkMaxSim(int numMotors, double gearing, double jKgMetersSquared) {
        DCMotor motor = DCMotor.getFalcon500(numMotors);

        motorSim = new DCMotorSim(motor, gearing, jKgMetersSquared);
    }

    public void update(double timestampSeconds) {
        motorSim.update(timestampSeconds - lastTimestampSeconds);

        lastTimestampSeconds = timestampSeconds;
    }

    public SparkMaxSim setController(PIDController controller) {
        this.controller = controller;
        return this;
    }

    public SparkMaxSim setProfiledController(ProfiledPIDController profiledController) {
        this.profiledController = profiledController;
        return this;
    }

    public double getBusVoltage() {
        return voltageRequest;
    }

    public double getAppliedOutput() {
        return voltageRequest / 12.0;
    }

    public double getVelocity() {
        return motorSim.getAngularVelocityRPM();
    }

    public double getPosition() {
        return motorSim.getAngularPositionRotations();
    }

    public double getOutputCurrent() {
        return motorSim.getCurrentDrawAmps();
    }

    public void set(double speed) {
        setInputVoltage(speed * 12.0);
    }

    public void setInputVoltage(double voltage) {
        voltageRequest = voltage;
        motorSim.setInputVoltage(voltageRequest);
    }

    public REVLibError setReference(double value, CANSparkMax.ControlType ctrl) {
        return setReference(value, ctrl, 0);
    }

    public REVLibError setReference(double value, CANSparkMax.ControlType ctrl, double arbFeedforward) {
        switch (ctrl) {
            case kDutyCycle:
                set(value);
                break;
            case kPosition:
                setInputVoltage(controller.calculate(getPosition(), value) + arbFeedforward);
                break;
            case kSmartMotion:
                setInputVoltage(profiledController.calculate(getPosition(), value) + arbFeedforward);
                break;
            case kVelocity:
                setInputVoltage(controller.calculate(getVelocity(), value) + arbFeedforward);
                break;
            case kSmartVelocity:
                setInputVoltage(profiledController.calculate(getVelocity(), value) + arbFeedforward);
                break;
            case kVoltage:
                setInputVoltage(value);
                break;
            case kCurrent:
                System.out.println("Can't use current control for spark max in sim!");
                break;
        }
        return REVLibError.kOk;
    }
}
