package frc.robot.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;

public interface ModuleIO {
    void updateInputs(SwerveModuleInputs inputs);

    Rotation2d getAngle();

    void setAngle(Rotation2d angle);

    double getVelocity();

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
