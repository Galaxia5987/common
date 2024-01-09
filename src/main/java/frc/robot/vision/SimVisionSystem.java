package frc.robot.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;

public class SimVisionSystem {
    private static SimVisionSystem INSTANCE;
    private static VisionSystemSim visionSim = new VisionSystemSim("main");
    private AprilTagFieldLayout tagFieldLayout;

    private SimVisionSystem(){
        try {
            tagFieldLayout =
                    AprilTagFieldLayout.loadFromResource(
                            AprilTagFields.k2023ChargedUp.m_resourceFile);
        } catch (Exception e) {
            return;
        }
        visionSim.addAprilTags(tagFieldLayout);
    }

    public static SimVisionSystem getInstance(PhotonCameraSim cameraSim, Transform3d robotToCam) {
        if (INSTANCE == null) {
            INSTANCE = new SimVisionSystem();
        }
        visionSim.addCamera(cameraSim, robotToCam);
        return INSTANCE;
    }
    public void update(Pose3d robotPose){
        visionSim.update(robotPose);
        visionSim.getDebugField();
    }
}
