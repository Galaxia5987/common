package frc.robot.conveyor;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Conveyor extends SubsystemBase {

    private static Conveyor INSTANCE = null;

    private final CANSparkMax motor;

    private Conveyor() {
        motor = new CANSparkMax(7, CANSparkMaxLowLevel.MotorType.kBrushless);

        motor.restoreFactoryDefaults();
        motor.setInverted(false);
        motor.setSmartCurrentLimit(40, 40);
        motor.burnFlash();
    }

    public static Conveyor getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new Conveyor();
        }
        return INSTANCE;
    }

    public void setPower(double power) {
        motor.set(power);
    }

    public Command spin(double spinPower) {
        return new RunCommand(() -> motor.set(spinPower), this)
                .withName("SpinConveyor");
    }
}
