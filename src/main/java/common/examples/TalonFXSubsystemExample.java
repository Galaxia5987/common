package common.examples;

import com.ctre.phoenix6.controls.PositionDutyCycle;
import edu.wpi.first.math.geometry.Rotation2d;
import lib.motors.TalonFXMotor;
import lib.motors.TalonFXReal;
import lib.motors.TalonFXSim;
import lib.units.Phoenix6UnitModel;

public class TalonFXSubsystemExample {

    private final TalonFXMotor motor;
    private final Phoenix6UnitModel unitModel;

    public TalonFXSubsystemExample(boolean isReal) {
        if (isReal) {
            motor = new TalonFXSim(1, 1, 0.001);
        } else {
            motor = new TalonFXReal(1);
        }

        unitModel = new Phoenix6UnitModel(30);
    }

    public void setAngle(Rotation2d desiredAngle) {
        double motorAngle = unitModel.getMotorAngle(desiredAngle).getRotations();
        motor.setControl(new PositionDutyCycle(motorAngle));
    }

    public Rotation2d getAngle() {
        return unitModel.getSystemAngle(motor.getRotorPosition());
    }
}
