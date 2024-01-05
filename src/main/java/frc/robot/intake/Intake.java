package frc.robot.intake;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.swerve.SwerveConstants;
import lib.Utils;

public class Intake extends SubsystemBase {

    private static Intake INSTANCE = null;

    private final CANSparkMax spinMotor;
//    private final TalonFX spinMotor;
    private final CANSparkMax angleMotor;

    private final TalonSRX turretMotor;

    private Intake() {
        spinMotor = new CANSparkMax(12, CANSparkMaxLowLevel.MotorType.kBrushless);
//        spinMotor = new TalonFX(12);
        angleMotor = new CANSparkMax(9, CANSparkMaxLowLevel.MotorType.kBrushless);

        turretMotor = new TalonSRX(12);

        spinMotor.restoreFactoryDefaults();
        spinMotor.setInverted(false);
        spinMotor.setSmartCurrentLimit(40, 40);
        spinMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        spinMotor.burnFlash();

//        spinMotor.configFactoryDefault();
//        spinMotor.setInverted(false);
//        spinMotor.configSupplyCurrentLimit(SwerveConstants.SUPPLY_CURRENT_LIMIT);
//        spinMotor.configStatorCurrentLimit(SwerveConstants.STATOR_CURRENT_LIMIT);
//        spinMotor.setNeutralMode(NeutralMode.Coast);

        angleMotor.restoreFactoryDefaults();
        angleMotor.setInverted(false);
        angleMotor.setSmartCurrentLimit(40, 40);
        angleMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        angleMotor.burnFlash();

        turretMotor.configFactoryDefault();
        turretMotor.setInverted(false);
        turretMotor.configSupplyCurrentLimit(SwerveConstants.SUPPLY_CURRENT_LIMIT);
        turretMotor.setNeutralMode(NeutralMode.Brake);
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

    public void setTurretSpeed(double speed){
        turretMotor.set(TalonSRXControlMode.PercentOutput, speed);
    }
}
