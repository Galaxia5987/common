package frc.robot.vision.heatMap;

import static frc.robot.vision.heatMap.HeatMapConstants.*;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.vision.VisionModule;
import java.util.Arrays;
import lib.Utils;

public class HeatMap {
    // 8m x 4m
    // Square size: 0.5m * 0.5m
    private VisionModule visionModule;
    private static HeatMap INSTANCE = null;

    /**
     * 2d Array that contains how many times the robot has entered each grid
     */
    private int[][] entryCounter =
            new int[(int) (xLength / squareLength)][(int) (yLength / squareLength)];
    /**
     * 2d Array that contains the average ambiguity for the camera in each grid
     */
    private double[][] fieldArr =
            new double[(int) (xLength / squareLength)][(int) (yLength / squareLength)];

    /**
     * Constructor for the Heatmap class
     * Initialize fieldArr values to be Integer max values
     * so the average ambiguity calculation will ignore them when calculating the average
     */
    private HeatMap() {
        for (double[] doubles : fieldArr) {
            Arrays.fill(doubles, Integer.MAX_VALUE);
        }
    }

    /**
     * @param visionModule visionModule object
     * @return Heatmap instance
     */
    public static HeatMap getInstance(VisionModule visionModule) {
        if (INSTANCE == null) {
            INSTANCE = new HeatMap();
        }
        INSTANCE.visionModule = visionModule;
        return INSTANCE;
    }

    /**
     * @return heatmap instance without passing anything
     */
    public static HeatMap getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new HeatMap();
        }
        return INSTANCE;
    }

    /**
     * Translates the robot pose to a grid pose
     * @param robotPose Pose3d of the robot current location
     * @return estimated robot pose on the field grid
     */
    public Pair<Integer, Integer> poseToGrid(Pose3d robotPose) {
        // Calculate the grid coordinates by dividing X and Y positions by squareLength
        Pair<Double, Double> gridPair =
                new Pair<>(robotPose.getX() / squareLength, robotPose.getY() / squareLength);

        // Round the grid coordinates to the nearest integers using Math.ceil
        Pair<Integer, Integer> roundedGridPair =
                new Pair<>(
                        ((int) Math.floor(gridPair.getFirst()) - 1),
                        ((int) Math.floor(gridPair.getSecond())) - 1);
        return roundedGridPair;
    }

    /**
     * Translates grid pose back to a Pose3d robot pose
     * @param grid robot grid location in Integer pairs
     * @param pitch camera pitch (rads)
     * @param height camera height (meters)
     * @param angle robot angle (rads)
     * @return Camera pose
     */
    public Pose3d gridToPose(
            Pair<Integer, Integer> grid, double pitch, double height, double angle) {
        return new Pose3d(
                (grid.getFirst() * squareLength),
                (grid.getSecond() * squareLength),
                height,
                new Rotation3d(0, pitch, angle));
    }

    /**
     * Function to reset the entry counter and the fieldArr
     * after every change in the camera settings
     */
    public void resetHeatMap() {
        entryCounter = new int[(int) (xLength / squareLength)][(int) (yLength / squareLength)];
        fieldArr = new double[(int) (xLength / squareLength)][(int) (yLength / squareLength)];

        for (double[] doubles : fieldArr) {
            Arrays.fill(doubles, Integer.MAX_VALUE);
        }
    }

    /**
     * @return 2d arr of average ambiguities in the grid
     */
    public double[][] getFieldArr() {
        return fieldArr;
    }

    /**
     *  Update function for the heatmap class
     *  needs to be called repeatedly
     * @param robotPose Pose3d of the robot current location on the field
     */
    public void update(Pose3d robotPose) {
        Pair<Integer, Integer> currentGrid = poseToGrid(robotPose);
        double totalAverageAmbiguity = fieldArr[currentGrid.getFirst()][currentGrid.getSecond()];
        double currentAverageAmbiguity = INSTANCE.visionModule.inputs.averageAmbiguity;

        fieldArr[currentGrid.getFirst()][currentGrid.getSecond()] =
                Utils.continuousAverageAmbiguity(
                        totalAverageAmbiguity,
                        currentAverageAmbiguity,
                        entryCounter[currentGrid.getFirst()][currentGrid.getSecond()]);
        entryCounter[currentGrid.getFirst()][currentGrid.getSecond()] += 1;
    }
}
