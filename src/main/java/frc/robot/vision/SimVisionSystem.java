package frc.robot.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.swerve.SwerveDrive;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;

public class SimVisionSystem {
    private static SimVisionSystem INSTANCE;
    private static final VisionSystemSim visionSim = new VisionSystemSim("main");
    private AprilTagFieldLayout tagFieldLayout;

    private SimVisionSystem() {
        try {
            tagFieldLayout =
                    AprilTagFieldLayout.loadFromResource(
                            AprilTagFields.k2024Crescendo.m_resourceFile);
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

    public static SimVisionSystem getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new SimVisionSystem();
        }
        return INSTANCE;
    }

    public void update() {
        visionSim.update(SwerveDrive.getInstance().getBotPose());
        visionSim.getDebugField();
    }

    public void adjustCameraPose(VisionModule visionModule, double height, double pitch) {
        visionSim.adjustCamera(
                visionModule.io.getCameraSim(),
                new Transform3d(
                        visionSim.getCameraPose(visionModule.io.getCameraSim()).get().getX(),
                        visionSim.getCameraPose(visionModule.io.getCameraSim()).get().getY(),
                        height,
                        new Rotation3d(0, pitch, 0)));
    }
}
