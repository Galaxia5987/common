package frc.robot.vision.heatMap;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.vision.VisionModule;
import lib.Utils;
import java.util.Arrays;

import static frc.robot.vision.heatMap.HeatMapConstants.*;

public class HeatMap {
    // 8m x 4m
    // Square size: 0.5m * 0.5m
    private VisionModule visionModule;

    private static HeatMap INSTANCE = null;

    private int[][] entryCounter =
            new int[(int) (xLength / squareLength)][(int) (yLength / squareLength)];
    private double[][] fieldArr =
            new double[(int) (xLength / squareLength)][(int) (yLength / squareLength)];

    private HeatMap() {
        for (double[] doubles : fieldArr) {
            Arrays.fill(doubles, Integer.MAX_VALUE);
        }
    }

    public static HeatMap getInstance(VisionModule visionModule) {
        if (INSTANCE == null) {
            INSTANCE = new HeatMap();
        }
        INSTANCE.visionModule = visionModule;
        return INSTANCE;
    }

    public static HeatMap getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new HeatMap();
        }
        return INSTANCE;
    }

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

    public Pose3d gridToPose(
            Pair<Integer, Integer> grid, double pitch, double height, double angle) {
        return new Pose3d(
                (grid.getFirst() * squareLength),
                (grid.getSecond() * squareLength),
                height,
                new Rotation3d(0, pitch, angle));
    }

    public void resetHeatMap(){
        int[][] entryCounter =
                new int[(int) (xLength / squareLength)][(int) (yLength / squareLength)];
        double[][] fieldArr =
                new double[(int) (xLength / squareLength)][(int) (yLength / squareLength)];
    }
    public double[][] getFieldArr() {
        return fieldArr;
    }

    public void update(Pose3d robotPose) {
        Pair<Integer, Integer> currentGrid = poseToGrid(robotPose);
        entryCounter[currentGrid.getFirst()][currentGrid.getSecond()] += 1;
        double totalAverageAmbiguity = fieldArr[currentGrid.getFirst()][currentGrid.getSecond()];
        double currentAverageAmbiguity = visionModule.inputs.averageAmbiguity;

        fieldArr[currentGrid.getFirst()][currentGrid.getSecond()] =
                Utils.continuousAverageAmbiguity(
                        totalAverageAmbiguity,
                        currentAverageAmbiguity,
                        entryCounter[currentGrid.getFirst()][currentGrid.getSecond()]);
    }
}
