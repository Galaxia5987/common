package lib.motors;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class SimMotor {

    protected final DCMotorSim motorSim;
    protected PIDController controller = null;
    protected ProfiledPIDController profiledController = null;

    protected double lastTimestampSeconds = 0;
    protected double voltageRequest = 0;

    protected final double conversionFactor;

    public SimMotor(
            LinearSystem<N2, N1, N2> model,
            DCMotor motor,
            double gearing,
            double conversionFactor) {
        this.motorSim = new DCMotorSim(model, motor, gearing);
        this.conversionFactor = conversionFactor / gearing;
    }

    public SimMotor(
            DCMotor motor, double jKgMetersSquared, double gearing, double conversionFactor) {
        this(
                LinearSystemId.createDCMotorSystem(motor, jKgMetersSquared, gearing),
                motor,
                gearing,
                conversionFactor);
    }

    public void setController(PIDController controller) {
        this.controller = controller;
    }

    public void setProfiledController(ProfiledPIDController profiledController) {
        this.profiledController = profiledController;
    }

    public void update(double timestampSeconds) {
        motorSim.update(timestampSeconds - lastTimestampSeconds);

        lastTimestampSeconds = timestampSeconds;
    }
}
