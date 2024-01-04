package frc.robot.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lib.Utils;

public class Intake extends SubsystemBase {

    private static Intake INSTANCE = null;

    private final CANSparkMax spinMotor;
    private final CANSparkMax angleMotor;

    private Intake() {
        spinMotor = new CANSparkMax(12, CANSparkMaxLowLevel.MotorType.kBrushless);
        angleMotor = new CANSparkMax(9, CANSparkMaxLowLevel.MotorType.kBrushless);

        spinMotor.restoreFactoryDefaults();
        spinMotor.setInverted(false);
        spinMotor.setSmartCurrentLimit(40, 40);
        spinMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        spinMotor.burnFlash();

        angleMotor.restoreFactoryDefaults();
        angleMotor.setInverted(false);
        angleMotor.setSmartCurrentLimit(40, 40);
        angleMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        angleMotor.burnFlash();
    }

    public static Intake getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new Intake();
        }
        return INSTANCE;
    }

    public void setSpinPower(double power) {
        spinMotor.set(power);
    }

    public void setAnglePower(double power) {
        angleMotor.set(power);
    }

    public double getAngleAppliedCurrent() {
        return angleMotor.getOutputCurrent();
    }

    public Command spin(double spinPower) {
        return new RunCommand(() -> spinMotor.set(spinPower), this)
                .withName("SpinIntake");
    }
}
