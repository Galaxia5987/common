package frc.robot.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.swerve.SwerveDrive;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;

public class SimVisionSystem {
    private static SimVisionSystem INSTANCE;
    private static final VisionSystemSim visionSim = new VisionSystemSim("main");
    private AprilTagFieldLayout tagFieldLayout;
    private boolean useSwerve = true;
    private Pose2d robotPose;

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
    public void setUseSwerve(boolean isSwerve){
        useSwerve = isSwerve;
    }
    public void setRobotPose(Pose2d robotPose){
        this.robotPose = robotPose;
    }
    public Pose2d getRobotPose(){
        if (useSwerve){
            return SwerveDrive.getInstance().getBotPose();
        }
        return robotPose;
    }

    public void update() {
        visionSim.update(SwerveDrive.getInstance().getBotPose());
        visionSim.getDebugField();
    }

    public void adjustCameraPose(VisionModule visionModule, double height, double pitch) {
        Transform3d robotToCam = new Transform3d(
                visionSim.getCameraPose(visionModule.io.getCameraSim()).get().getX(),
                visionSim.getCameraPose(visionModule.io.getCameraSim()).get().getY(),
                height,
                new Rotation3d(0, pitch, 0));
        visionModule.io.setCameraPose(robotToCam);
        visionSim.adjustCamera(
                visionModule.io.getCameraSim(), robotToCam);
    }
}
