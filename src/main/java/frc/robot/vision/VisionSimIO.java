package frc.robot.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.swerve.SwerveDrive;
import lib.Utils;
import org.photonvision.PhotonCamera;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;
import org.photonvision.targeting.PhotonPipelineResult;
import java.io.IOException;

public class VisionSimIO implements VisionIO {
    private VisionSystemSim visionSim;
    private PhotonCamera photonCamera;
    private PhotonCameraSim cameraSim;
    private PhotonPipelineResult latestResult = new PhotonPipelineResult();
    private PhotonTrackedTarget trackedTarget = new PhotonTrackedTarget();
    private Result result;
    private List<VisionTargetSim> visionTargetsSim = new ArrayList<>();
    private Transform3d robotToCam;

    public VisionSimIO(Transform3d robotToCam) {
        this.robotToCam = robotToCam;
//        simVisionSystem = new VisionSystemSim("simCam", 95, robotToCam.getRotation().getY(), new Transform2d(robotToCam.getTranslation().toTranslation2d(), robotToCam.getRotation().toRotation2d()), robotToCam.getZ(), 1000, 1600, 1200, 0);
        photonCamera = new PhotonCamera(NetworkTableInstance.getDefault(), "photonCam");
        cameraSim = new PhotonCameraSim(photonCamera);
        try {
            var field = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
            field.getTags().forEach((tag) ->
                    visionTargetsSim.add(new VisionTargetSim(tag.pose, new TargetModel(0.15, 0.15))));
        } catch (Exception e) {
            e.printStackTrace();
        visionSim = new VisionSystemSim(photonCamera.getName());
        }

        visionSim.addCamera(cameraSim, robotToCam);
        visionSim.addAprilTags(tagFieldLayout);
    }


    @Override
    public void setPipeLine(int pipeLineIndex) {
//        camera.setPipelineIndex(pipeLineIndex);
    }

    @Override
    public Result getLatestResult() {
        return result;
    }

    @Override
    public void updateInputs(VisionInputs inputs) {
        var pose = SwerveDrive.getInstance().getBotPose();

        latestResult = cameraSim.process(0, new Pose3d(pose).plus(robotToCam), visionTargetsSim);
        inputs.latency = (long) latestResult.getLatencyMillis();
        inputs.hasTargets = latestResult.hasTargets();

        if (latestResult.getBestTarget() != null) {
            inputs.area = latestResult.getBestTarget().getArea();
            inputs.pitch = latestResult.getBestTarget().getPitch();
            inputs.yaw = latestResult.getBestTarget().getYaw();
            inputs.targetSkew = latestResult.getBestTarget().getSkew();
            inputs.targetID = latestResult.getBestTarget().getFiducialId();

            var cameraToTarget = latestResult.getBestTarget().getBestCameraToTarget();
            inputs.cameraToTarget = new double[]{
                    cameraToTarget.getX(),
                    cameraToTarget.getY(),
                    cameraToTarget.getZ(),
                    cameraToTarget.getRotation().getX(),
                    cameraToTarget.getRotation().getY(),
                    cameraToTarget.getRotation().getZ()
            };

            inputs.poseFieldOriented = new double[]{
                    pose.getX(),
                    pose.getY(),
                    0,
                    0,
                    0,
                    pose.getRotation().getRadians()
            };
        } else {
            result = null;
        }
    }

}
