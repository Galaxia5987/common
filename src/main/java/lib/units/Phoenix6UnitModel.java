package lib.units;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;

public class Phoenix6UnitModel {

    private final double gearRatio;

    /**
     * Constructor.
     *
     * @param gearRatio the ratio between the motor and the system.
     *                  If the gear ratio is larger than 1, it is a reduction.
     */
    public Phoenix6UnitModel(double gearRatio) {
        this.gearRatio = gearRatio;
    }

    public Rotation2d getSystemAngle(double revolutions) {
        return Rotation2d.fromRotations(revolutions / gearRatio);
    }

    public Rotation2d getSystemVelocity(double revolutionsPerSecond) {
        return Rotation2d.fromRotations(revolutionsPerSecond / gearRatio);
    }

    public double getSystemTorque(double current, DCMotor motor) {
        return current * motor.KtNMPerAmp * gearRatio;
    }

    public Rotation2d getMotorAngle(Rotation2d angle) {
        return angle.times(gearRatio);
    }

    public Rotation2d getMotorVelocity(Rotation2d velocity) {
        return velocity.times(gearRatio);
    }
}
