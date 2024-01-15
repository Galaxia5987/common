package frc.robot.vision.heatMap;

public class HeatMapField {
    private double[][] fieldArr;

    public HeatMapField(double[][] fieldArr) {
        this.fieldArr = fieldArr;
    }

    public double[][] getFieldArr() {
        return fieldArr;
    }

    public void setFieldArr(double[][] fieldArr) {
        this.fieldArr = fieldArr;
    }
}
