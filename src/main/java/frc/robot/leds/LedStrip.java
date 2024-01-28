package frc.robot.leds;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;

import java.awt.*;

public class LedStrip {
    private final AddressableLED ledStrip;
    private AddressableLEDBuffer ledBuffer;

    private double blinkTime = 1;
    private double fadeDuration = 1;
    private int rainbowHue = 0;

    private Color primary = new Color(0);
    private Color secondary = new Color(0);
    private Color currentColor = primary;

    private final Timer timer = new Timer();
    public Translation3d solution = new Translation3d();
    private Mode mode = Mode.SOLID;

    public LedStrip(int port, int length) {
        ledStrip = new AddressableLED(port);
        ledBuffer = new AddressableLEDBuffer(length);

        ledStrip.setLength(length);
        ledStrip.setData(ledBuffer);
        ledStrip.start();

        timer.start();
        timer.reset();
    }

    /**
     * Sets the length of the led strip.
     *
     * @param length Number of leds in the strip.
     */
    public void setLength(int length) {
        ledBuffer = new AddressableLEDBuffer(length);
        ledStrip.setLength(length);
    }

    /**
     * Sets the primary color of the strip.
     *
     * @param color Primary color.
     */
    public void setPrimary(Color color) {
        primary = color;
    }

    /**
     * Sets the secondary color of the strip.
     *
     * @param color Secondary color.
     */
    public void setSecondary(Color color) {
        secondary = color;
    }

    /**
     * Switch mode to solid color.
     */
    public void solid() {
        mode = Mode.SOLID;
    }

    /**
     * Switch mode to blink.
     * (Should use setBlinkTime before this)
     */
    public void blink() {
        mode = Mode.BLINK;
    }

    /**
     * Switch mode to fade.
     * (Should use setFadeDuration before this)
     */
    public void fade() {
        mode = Mode.FADE;
    }

    /**
     * Switch mode to rainbow.
     */
    public void rainbow() {
        mode = Mode.RAINBOW;
    }

    /**
     * Sets the led blink time.
     *
     * @param blinkTime Blink time in seconds
     */
    public void setBlinkTime(double blinkTime) {
        this.blinkTime = blinkTime;
    }

    /**
     * Sets the duration of the fade effect.
     *
     * @param duration Fade duration in seconds.
     */
    public void setFadeDuration(double duration) {
        fadeDuration = duration;
    }

    /**
     * Sets the led strip to a solid color
     *
     * @param color Color to set the strip to.
     */
    private void setSolidColor(Color color) {
        var appliedColor = new edu.wpi.first.wpilibj.util.Color(color.getRed(), color.getGreen(), color.getBlue());
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setLED(i, appliedColor);
        }
        ledStrip.setData(ledBuffer);
    }

    /**
     * Interpolates between two colors.
     *
     * @param initial Initial color.
     * @param goal    Final Color.
     * @return
     */
    private Translation3d colorInterpolation(Color initial, Color goal) {
        var initialHSB = Color.RGBtoHSB(initial.getRed(), initial.getGreen(), initial.getBlue(), new float[3]);
        var goalHSB = Color.RGBtoHSB(goal.getRed(), goal.getGreen(), goal.getBlue(), new float[3]);

        Translation3d initialPoint = new Translation3d(initialHSB[0], initialHSB[1], initialHSB[2]);
        Translation3d goalPoint = new Translation3d(goalHSB[0], goalHSB[1], goalHSB[2]);

        double d = initialPoint.getDistance(goalPoint) / fadeDuration;
        double t = d / (initialPoint.minus(goalPoint)).getNorm();

        solution = initialPoint.interpolate(goalPoint, t);
        return solution;
    }


    /**
     * Sets the led strip to a rainbow pattern.
     */
    private void setRainbow() {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setHSV(i, rainbowHue, 255, 180);
            rainbowHue += (180 / ledBuffer.getLength());
            ledStrip.setData(ledBuffer);
            rainbowHue %= 180;
        }
    }

    public void updateStrip() {
        if (mode == Mode.SOLID) {
            setSolidColor(currentColor);
        }
        if (mode == Mode.BLINK) {
            if (currentColor == primary) currentColor = secondary;
            else currentColor = primary;

            if (timer.hasElapsed(blinkTime)) {
                setSolidColor(currentColor);
            }
            timer.reset();
        }
        if (mode == Mode.FADE) {
            solution = colorInterpolation(primary, secondary);
            setSolidColor(new Color(
                    (int) solution.getX(),
                    (int) solution.getY(),
                    (int) solution.getZ()
            ));
        }
        if (mode == Mode.RAINBOW) {
            setRainbow();
        }
    }

    enum Mode {
        SOLID, BLINK, FADE, RAINBOW
    }
}
