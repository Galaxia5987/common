package frc.robot.vision;

import edu.wpi.first.math.geometry.*;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionTargetSim;

public class VisionConstants {
    public static final Transform3d[] ROBOT_TO_CAM = new Transform3d[]{
            new Transform3d( //left cam
                    new Translation3d(-0.249, 0.30397, 0.5023),
                    new Rotation3d(0, 0, Math.toRadians(142.5))),
            new Transform3d( //right cam
                    new Translation3d(-0.2363, -0.527, 0.196),
                    new Rotation3d(0, 28, Math.toRadians(-151)))};

    public static final Transform3d LIME_OFFSET = new Transform3d(new Translation3d( (16.54) / 2,(8.02) / 2, 0), new Rotation3d());
    public static final double TARGET_WIDTH = 0.27;
    public static final double TARGET_LENGTH = 0.27;
    public static final Pose3d[] TARGET_POSITION_REAL = new Pose3d[]{new Pose3d() ,new Pose3d(new Translation3d( 7.24310, -2.93659, 0.46272), new Rotation3d(0,0,180)),new Pose3d(new Translation3d(7.24310, 1.26019, 0.46272), new Rotation3d(0,0,180)), new Pose3d(new Translation3d(7.24310, 0.41621, 0.41621), new Rotation3d(0,0,180)), new Pose3d(new Translation3d(7.90832, 2.74161, 0.695452), new Rotation3d(0,0,180)), new Pose3d(new Translation3d(-7.90832, 2.74161, 0.695452), new Rotation3d()), new Pose3d(new Translation3d(-7.24310, 0.41621, 0.46272), new Rotation3d()), new Pose3d(new Translation3d(-7.24310, 1.26019, 0.46272), new Rotation3d()), new Pose3d(new Translation3d(-7.24310, -2.93659, 0.46272), new Rotation3d())};
    public static final Pose3d[] TARGET_POSITION_LIME = new Pose3d[]{new Pose3d() ,new Pose3d(new Translation3d( 7.24310, -2.93659, 0.46272), new Rotation3d(0,0,180)),new Pose3d(new Translation3d(7.24310, 1.26019, 0.46272), new Rotation3d(0,0,180)), new Pose3d(new Translation3d(7.24310, 0.41621, 0.41621), new Rotation3d(0,0,180)), new Pose3d(new Translation3d(7.90832, 2.74161, 0.695452), new Rotation3d(0,0,180)), new Pose3d(new Translation3d(-7.90832, 2.74161, 0.695452), new Rotation3d()), new Pose3d(new Translation3d(-7.24310, 0.41621, 0.46272), new Rotation3d()), new Pose3d(new Translation3d(-7.24310, 1.26019, 0.46272), new Rotation3d()), new Pose3d(new Translation3d(-7.24310, -2.93659, 0.46272), new Rotation3d())};
    public static final Pose3d[] TARGET_POSITION_SIM = new Pose3d[]{new Pose3d() ,new Pose3d(new Translation3d( 7.24310, -2.93659, 0.46272), new Rotation3d(0,0,180)),new Pose3d(new Translation3d(7.24310, 1.26019, 0.46272), new Rotation3d(0,0,180)), new Pose3d(new Translation3d(7.24310, 0.41621, 0.41621), new Rotation3d(0,0,180)), new Pose3d(new Translation3d(7.90832, 2.74161, 0.695452), new Rotation3d(0,0,180)), new Pose3d(new Translation3d(-7.90832, 2.74161, 0.695452), new Rotation3d()), new Pose3d(new Translation3d(-7.24310, 0.41621, 0.46272), new Rotation3d()), new Pose3d(new Translation3d(-7.24310, 1.26019, 0.46272), new Rotation3d()), new Pose3d(new Translation3d(-7.24310, -2.93659, 0.46272), new Rotation3d())};

    public static final VisionTargetSim[] VISION_TARGETS_SIM = new VisionTargetSim[]{};
    public static final SimCameraProperties simCameraProperties = new SimCameraProperties(
        //TODO:Add sim camera properties
    );
}
