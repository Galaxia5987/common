package utils.motors;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class GalacticTalonFX extends TalonFX {

    public static final int CONFIG_TIMEOUT = 100;

    public static final StatorCurrentLimitConfiguration LOW_CURRENT_LIMIT = new StatorCurrentLimitConfiguration(
            true, 30, 0, 0);
    public static final StatorCurrentLimitConfiguration MEDIUM_CURRENT_LIMIT = new StatorCurrentLimitConfiguration(
            true, 40, 0, 0);
    public static final StatorCurrentLimitConfiguration HIGH_CURRENT_LIMIT = new StatorCurrentLimitConfiguration(
            true, 50, 0, 0);

    public GalacticTalonFX(int deviceNumber, String canbus) {
        super(deviceNumber, canbus);
    }

    public GalacticTalonFX(int deviceNumber) {
        super(deviceNumber);
    }

    public GalacticTalonFX setPIDF(double p, double i, double d, double f) {
        this.config_kP(0, p, CONFIG_TIMEOUT);
        this.config_kI(0, i, CONFIG_TIMEOUT);
        this.config_kD(0, d, CONFIG_TIMEOUT);
        this.config_kF(0, f, CONFIG_TIMEOUT);
        return this;
    }

    public GalacticTalonFX setMotionMagic(double cruiseVelocity, double acceleration) {
        this.configMotionCruiseVelocity(cruiseVelocity, CONFIG_TIMEOUT);
        this.configMotionAcceleration(acceleration, CONFIG_TIMEOUT);
        return this;
    }

    public GalacticTalonFX setAllMotionControl(double p, double i, double d, double f,
                                               double cruiseVelocity, double acceleration,
                                               int sCurveStrength, double allowableError,
                                               double maxIntegralAccumulator, double closedLoopPeakOutput) {
        this.setPIDF(p, i, d, f);
        this.setMotionMagic(cruiseVelocity, acceleration);
        this.configMotionSCurveStrength(sCurveStrength, CONFIG_TIMEOUT);
        this.configAllowableClosedloopError(0, allowableError, CONFIG_TIMEOUT);
        this.configMaxIntegralAccumulator(0, maxIntegralAccumulator, CONFIG_TIMEOUT);
        this.configClosedLoopPeakOutput(0, closedLoopPeakOutput, CONFIG_TIMEOUT);
        return this;
    }

    public GalacticTalonFX setLowCurrentLimit() {
        this.configStatorCurrentLimit(LOW_CURRENT_LIMIT, CONFIG_TIMEOUT);
        return this;
    }

    public GalacticTalonFX setMediumCurrentLimit() {
        this.configStatorCurrentLimit(MEDIUM_CURRENT_LIMIT, CONFIG_TIMEOUT);
        return this;
    }

    public GalacticTalonFX setHighCurrentLimit() {
        this.configStatorCurrentLimit(HIGH_CURRENT_LIMIT, CONFIG_TIMEOUT);
        return this;
    }

    public GalacticTalonFX setVoltageCompensation() {
        this.enableVoltageCompensation(true);
        this.configVoltageCompSaturation(12, CONFIG_TIMEOUT);
        return this;
    }
}
