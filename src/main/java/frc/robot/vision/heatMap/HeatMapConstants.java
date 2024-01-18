package frc.robot.vision.heatMap;

public class HeatMapConstants {
    /**
     * Constant for the minimum height of the camera
     * Units are in meters
     */
    public static double heightMaximumRange = 0.8; // m
    /**
     * Constant for the maximum height of the camera
     * Units are in meters
     */
    public static double heightMinimumRange = 0.2; // m
    /**
     * Constant for the increase in camera height
     * Used after the robot has gathered data in all the wanted grids
     * Units are in meters
     */
    public static double heightDelta = 0.05; // m
    /**
     * Constant for the change in robot angle
     * Units are in rads
     */
    public static double robotAngleDelta = 5; // deg
    /**
     * Estimated runtime for the TestCameras execute function
     * Units are in seconds
     */
    public static double iterationTime = 0.04; // s

    /**
     * Length of the grid square of the field
     * Units are in meters
     */
    public static final double squareLength = 0.5; // m
    /**
     * Length of the y the robot checks in the field
     * Units are in meters
     */
    public static final double yLength = 8; // m
    /**
     * Length of the x the robot checks in the field
     * Units are in meters
     */
    public static final double xLength = 4; // m
}
