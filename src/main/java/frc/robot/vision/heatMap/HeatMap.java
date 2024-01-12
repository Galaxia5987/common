package frc.robot.vision.heatMap;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.vision.VisionModule;
import lib.Utils;

public class HeatMap {
    // 16m x 8m
    // Square size: 0.5m * 0.5m
    private VisionModule visionModule;

    private static HeatMap INSTANCE = null;

    private final double squareLength = 0.5; // m
    private final double xLength = 16; // m
    private final double yLength = 8; // m

    private int[][] entryCounter =
            new int[(int) (xLength / squareLength)][(int) (yLength / squareLength)];
    private double[][] fieldArr =
            new double[(int) (xLength / squareLength)][(int) (yLength / squareLength)];
    private Pair<Integer, Integer> lastGrid;

    private HeatMap() {}

    public static HeatMap getInstance(VisionModule visionModule) {
        if (INSTANCE == null) {
            INSTANCE = new HeatMap();
        }
        INSTANCE.visionModule = visionModule;
        return INSTANCE;
    }

    public static HeatMap getInstance(){
        if (INSTANCE == null){
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
                        ((int) Math.ceil(gridPair.getFirst())),
                        ((int) Math.ceil(gridPair.getSecond())));
        return roundedGridPair;
    }

    public boolean hasPassed(Pair<Integer, Integer> currentGrid) {
        boolean passed = currentGrid.equals(lastGrid);

        if (passed) {
            entryCounter[currentGrid.getFirst()][currentGrid.getSecond()] += 1;
        }
        lastGrid = currentGrid;

        return passed;
    }

    public void update(Pose3d robotPose) {
        Pair<Integer, Integer> gridPose = poseToGrid(robotPose);
        if (hasPassed(gridPose)) {
            double totalAverageAmbiguity = fieldArr[gridPose.getFirst()][gridPose.getSecond()];
            double currentAverageAmbiguity = visionModule.inputs.averageAmbiguity;

            fieldArr[gridPose.getFirst()][gridPose.getSecond()] =
                    Utils.continousAverageAmbiguity(
                            totalAverageAmbiguity,
                            currentAverageAmbiguity,
                            entryCounter[gridPose.getFirst()][gridPose.getSecond()]);
        }
    }
}
