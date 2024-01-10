package frc.robot.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.swerve.SwerveDrive;
import lib.Utils;
import org.photonvision.PhotonCamera;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionTargetSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionSimIO implements VisionIO {
    private SimVisionSystem simVisionSystem;
    private final PhotonCamera photonCamera;
    private final PhotonCameraSim cameraSim;
    private final Transform3d robotToCam;
    private Result result;
    private AprilTagFieldLayout tagFieldLayout;

    public VisionSimIO(PhotonCamera photonCamera, Transform3d robotToCam) {
        this.robotToCam = robotToCam;
        this.photonCamera = photonCamera;
        cameraSim = new PhotonCameraSim(photonCamera, VisionConstants.simCameraProperties);
        cameraSim.enableRawStream(true);
        cameraSim.enableProcessedStream(true);
        cameraSim.enableDrawWireframe(true);
        try {
            tagFieldLayout =
                    AprilTagFieldLayout.loadFromResource(
                            AprilTagFields.k2023ChargedUp.m_resourceFile);
        } catch (Exception e) {
            return;
        }
        simVisionSystem = SimVisionSystem.getInstance(cameraSim, robotToCam);
    }

    @Override
    public void setPipeLine(int pipeLineIndex) {}

    @Override
    public Result getLatestResult() {
        return result;
    }

    @Override
    public Transform3d getCameraToRobot() {
        return robotToCam;
    }

    @Override
    public String getName() {
        return photonCamera.getName();
    }

    @Override
    public void updateInputs(VisionInputs inputs) {
        var pose = SwerveDrive.getInstance().getBotPose();
        var pose3d = Utils.pose2dToPose3d(pose);

        simVisionSystem.update();
        PhotonPipelineResult latestResult =
                cameraSim.process(
                        0,
                        pose3d.plus(robotToCam.div(-1)),
                        tagFieldLayout.getTags().stream()
                                .map(
                                        (a) ->
                                                new VisionTargetSim(
                                                        a.pose, TargetModel.kAprilTag36h11, a.ID))
                                .toList());
        inputs.latency = (long) latestResult.getLatencyMillis();
        inputs.hasTargets = latestResult.hasTargets();
        if (inputs.hasTargets) {
            PhotonTrackedTarget bestTarget = latestResult.getBestTarget();
            if (bestTarget != null) {
                inputs.area = bestTarget.getArea();
                inputs.pitch = bestTarget.getPitch();
                inputs.yaw = bestTarget.getYaw();
                inputs.targetSkew = bestTarget.getSkew();
                inputs.targetID = bestTarget.getFiducialId();
                inputs.targetAmbiguity = bestTarget.getPoseAmbiguity();

                var cameraToTarget = bestTarget.getBestCameraToTarget();
                inputs.cameraToTarget =
                        new double[] {
                            cameraToTarget.getX(),
                            cameraToTarget.getY(),
                            cameraToTarget.getZ(),
                            cameraToTarget.getRotation().getX(),
                            cameraToTarget.getRotation().getY(),
                            cameraToTarget.getRotation().getZ()
                        };
            }
        }
    }
}
