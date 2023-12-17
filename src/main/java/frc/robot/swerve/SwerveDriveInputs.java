package frc.robot.swerve;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class SwerveDriveInputs implements LoggableInputs {
    public double supplyCurrent;
    public double statorCurrent;

    public double[] desiredModuleStates = new double[8];

    // x, y, omega
    public double[] currentSpeeds = {0, 0, 0};
    public double[] desiredSpeeds = {0, 0, 0};

    public double linearVelocity = 0;
    public double acceleration = 0;

    public double[] absolutePositions = new double[4];

    public double pitch;
    public double rawYaw;
    public double yaw;
    public double gyroOffset;

    public double[] botPose = {0, 0, 0}; //x, y, rotation

    @Override
    public void toLog(LogTable table) {
        table.put("supplyCurrent", supplyCurrent);
        table.put("statorCurrent", statorCurrent);

        table.put("desiredModuleStates", desiredModuleStates);

        table.put("currentSpeeds", currentSpeeds);
        table.put("desiredSpeeds", desiredSpeeds);

        table.put("linearVelocity", linearVelocity);
        table.put("acceleration", acceleration);

        table.put("absolutePositions", absolutePositions);

        table.put("pitch", pitch);
        table.put("rawYaw", rawYaw);
        table.put("yaw", yaw);
        table.put("gyroOffset", gyroOffset);

        table.put("botPose", botPose);
    }

    @Override
    public void fromLog(LogTable table) {
        table.get("supplyCurrent", supplyCurrent);
        table.get("statorCurrent", statorCurrent);

        table.get("desiredModuleStates", desiredModuleStates);

        table.get("currentSpeeds", currentSpeeds);
        table.get("desiredSpeeds", desiredSpeeds);

        table.get("linearVelocity", linearVelocity);
        table.get("acceleration", acceleration);

        table.get("absolutePositions", absolutePositions);

        table.get("pitch", pitch);
        table.get("rawYaw", rawYaw);
        table.get("yaw", yaw);
        table.get("gyroOffset", gyroOffset);

        table.get("botPose", botPose);
    }
}
