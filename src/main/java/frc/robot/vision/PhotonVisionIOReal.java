package frc.robot.vision;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

public class PhotonVisionIOReal implements VisionIO {

    private final PhotonCamera camera;
    private final PhotonPoseEstimator estimator;
    private final Transform3d robotToCamera;
    private Result result;

    public PhotonVisionIOReal(PhotonCamera camera, Transform3d robotToCamera) {
        this.camera = camera;
        this.robotToCamera = robotToCamera;
        camera.setPipelineIndex(0);
        try {
            estimator = new PhotonPoseEstimator(
                    AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField(),
                    PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                    camera,
                    robotToCamera
            );
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
    }

    @Override
    public void setPipeLine(int pipeLineIndex) {
        camera.setPipelineIndex(pipeLineIndex);
    }

    @Override
    public void updateInputs(VisionInputs inputs) {
        var latestResult = camera.getLatestResult();

        if (latestResult != null) {//TODO: check if the value can be null
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
            }

            var estimatedPose = estimator.update(latestResult);
            if (estimatedPose.isPresent()) {
                var pose = estimatedPose.get().estimatedPose;
                inputs.poseFieldOriented = new double[]{
                        pose.getX(),
                        pose.getY(),
                        pose.getZ(),
                        pose.getRotation().getX(),
                        pose.getRotation().getY(),
                        pose.getRotation().getZ()
                };

                result = new Result(latestResult.getTimestampSeconds(), estimatedPose.get().estimatedPose);
            } else {
                result = null;
            }
        } else {
            result = null;
        }
    }

    @Override
    public Result getLatestResult() {
        return result;
    }
    @Override
    public Transform3d getCameraToRobot() {
        return robotToCamera;
    }
}
