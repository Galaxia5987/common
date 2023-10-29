package utils.webconstants;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;

/*
This class contains a constant that can be changed from the web dashboard.
 */
public class WebConstant {

    // The list of all WebConstants
    private static final ArrayList<WebConstant> INSTANCES = new ArrayList<>();

    // The name of the constant
    public final String name;
    // The value of the constant
    public double value;

    // The entry in the web dashboard
    public NetworkTableEntry entry;

    /**
     * Constructor for WebConstant.
     * @param name The name of the constant.
     * @param defaultValue The default value of the constant.
     */
    public WebConstant(String name, double defaultValue) {
        this.name = name;
        this.value = defaultValue;
        INSTANCES.add(this);
        entry = NetworkTableInstance.getDefault().getEntry("Web Constants/" + name);
    }

    /**
     * Gets the value of the constant.
     * @return The value of the constant.
     */
    public double get() {
        return value;
    }

    /**
     * Sets the value of the constant.
     * @param value The new value of the constant.
     */
    public void set(double value) {
        this.value = value;
        if (entry != null) {
            entry.setDouble(value);
        }
    }

    /**
     * Updates the constant.
     */
    private void update() {
        value = entry.getDouble(value);
    }

    /**
     * Updates all WebConstants.
     */
    public static void updateAll() {
        INSTANCES.forEach(WebConstant::update);
    }
}
