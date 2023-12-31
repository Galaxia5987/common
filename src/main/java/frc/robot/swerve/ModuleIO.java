package frc.robot.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
    void updateInputs(SwerveModuleInputs inputs);

    default Rotation2d getAngle() {
        return new Rotation2d();
    }

    void setAngle(Rotation2d angle);

    default double getVelocity() {
        return 0;
    }

    void setVelocity(double velocity);

    SwerveModuleState getModuleState();

    default SwerveModulePosition getModulePosition(){
        return null;
    }

    default void updateOffset(double offset) {
    }

    default void stopMotor() {
    }

    default boolean encoderConnected() {
        return false;
    }

    default Command checkModule() {
        return null;
    }
}
