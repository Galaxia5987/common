package frc.robot.vision;

import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {


    void setPipeLine(int pipeLineIndex);

    void updateInputs(VisionInputs inputs);

    Result getLatestResult();

    @AutoLog
    class VisionInputs {
        long latency = 0;
        boolean hasTargets = false;
        double yaw = 0;
        double pitch = 0;
        double area = 0;
        double targetSkew = 0;
        long targetID = 0;
        double[] cameraToTarget = new double[]{0,0,0,0,0,0};
        double[] poseFieldOriented = new double[6];
    }
}
