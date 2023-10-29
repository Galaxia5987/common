package utils.webconstants;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

import java.util.ArrayList;

/*
This class contains a constant that can be changed from the web dashboard.
 */
public class WebConstant {

    // The list of all WebConstants
    private static final ArrayList<WebConstant> INSTANCES = new ArrayList<>();
    // Whether the robot was in debug mode last loop
    private static boolean lastDebug = false;

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
        if (Robot.debug) {
            entry = NetworkTableInstance.getDefault().getEntry("Web Constants/" + name);
        } else {
            entry = null;
        }
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
        if (Robot.debug && !lastDebug) {
            entry = NetworkTableInstance.getDefault().getEntry("Web Constants/" + name);
        }
        if (!Robot.debug && lastDebug) {
            entry.close();
            entry = null;
        }
        if (entry != null) {
            value = entry.getDouble(value);
        }
    }

    /**
     * Updates all WebConstants.
     */
    public static void updateAll() {
        lastDebug = Robot.debug;
        Robot.debug = SmartDashboard.getBoolean("debug", false);
        INSTANCES.forEach(WebConstant::update);
    }
}
