package frc.robot.vision.heatMap;

import edu.wpi.first.util.struct.Struct;
import java.nio.ByteBuffer;

public class HeatMapFieldStruct implements Struct<HeatMapField> {

    @Override
    public Class<HeatMapField> getTypeClass() {
        return HeatMapField.class;
    }

    @Override
    public String getTypeString() {
        return "HeatMapField";
    }

    @Override
    public int getSize() {
        return -1;
    }

    @Override
    public String getSchema() {
        return "double x[]; double y[];";
    }

    @Override
    public HeatMapField unpack(ByteBuffer bb) {
        int rows = bb.getInt();
        int columns = bb.getInt();

        double[][] result = new double[rows][columns];

        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < columns; j++) {
                result[i][j] = bb.getDouble();
            }
        }
        return new HeatMapField(result);
    }

    @Override
    public void pack(ByteBuffer bb, HeatMapField value) {
        double[][] data = value.getFieldArr();

        int rows = data.length;
        int columns = (rows > 0) ? data[0].length : 0; // if the arr is empty
        bb.putInt(rows);
        bb.putInt(columns);

        for (double[] datum : data) {
            for (int j = 0; j < columns; j++) {
                bb.putDouble(datum[j]);
            }
        }
    }
}
