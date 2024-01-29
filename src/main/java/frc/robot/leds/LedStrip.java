package frc.robot.leds;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.awt.*;

public class LedStrip extends SubsystemBase {
    private final AddressableLED ledStrip;
    private AddressableLEDBuffer ledBuffer;
    private int stripLength = 0;

    private double blinkTime = 1;
    private double fadeDuration = 1;

    private Color primary = new Color(0);
    private Color secondary = new Color(0);
    private Color currentColor = primary;
    private Color fadeColor = primary;

    private final Timer timer = new Timer();
    private LedMode mode = LedMode.SOLID;

    public LedStrip(int port, int length) {
        ledStrip = new AddressableLED(port);
        ledBuffer = new AddressableLEDBuffer(length);
        stripLength = length;

        ledStrip.setLength(length);
        ledStrip.setData(ledBuffer);
        ledStrip.start();

        timer.start();
        timer.reset();
    }

    public void setLength(int length) {
        ledBuffer = new AddressableLEDBuffer(length);
        ledStrip.setLength(length);
        stripLength = length;
    }

    public void setState(LedState state){
        setBlinkTime(state.getBlinkTime());
        setFadeDuration(state.getFadeDuration());
        setMode(state.getMode());
        setPrimary(state.getPrimary());
        setSecondary(state.getSecondary());
    }

    /**
     * Sets the primary color of the LEDs.
     * It will be used when in the solid mode and as the
     * initial color in the blink and fade modes
     * @param color
     */
    private void setPrimary(Color color) {
        primary = color;
    }

    /**
     * Sets the secondary color of the LEDs.
     * It will be used as the second color in the blink and fade modes
     * @param color
     */
    private void setSecondary(Color color) {
        secondary = color;
    }

    private void setMode(LedMode mode) {
        this.mode = mode;
    }

    /**
     * Sets the led blink time.
     * @param blinkTime Blink time in seconds
     */
    private void setBlinkTime(double blinkTime) {
        this.blinkTime = blinkTime;
    }

    /**
     * Sets the duration of the fade effect.
     * @param duration Fade duration in seconds.
     */
    private void setFadeDuration(double duration) {
        fadeDuration = duration;
    }

    private void setSolidColor(Color color) {
        var appliedColor = new edu.wpi.first.wpilibj.util.Color(color.getRed(), color.getGreen(), color.getBlue());
        for (int i = 0; i < stripLength; i++) {
            ledBuffer.setLED(i, appliedColor);
        }
        ledStrip.setData(ledBuffer);
    }

    /**
     * Updates fadeColor to the correct color for a fade effect
     * at the current time by interpolating the two given colors.
     * @param initial Initial color.
     * @param goal    Final Color.
     */
    private void colorInterpolation(Color initial, Color goal) {
        var initialHSB = Color.RGBtoHSB(initial.getRed(), initial.getGreen(), initial.getBlue(), new float[3]);
        var goalHSB = Color.RGBtoHSB(goal.getRed(), goal.getGreen(), goal.getBlue(), new float[3]);

        Translation3d initialPoint = new Translation3d(initialHSB[0], initialHSB[1], initialHSB[2]);
        Translation3d goalPoint = new Translation3d(goalHSB[0], goalHSB[1], goalHSB[2]);

        double d = initialPoint.getDistance(goalPoint) / fadeDuration;
        double t = d / (initialPoint.minus(goalPoint)).getNorm();

        Translation3d solution = initialPoint.interpolate(goalPoint, t);
        fadeColor = new Color(
                (int)solution.getX(),
                (int)solution.getY(),
                (int)solution.getZ()
        );
    }

    private void setRainbow() {
        int rainbowHue = 0;
        for (int i = 0; i < stripLength; i++) {
            ledBuffer.setHSV(i, rainbowHue, 255, 180);
            rainbowHue += (180 / stripLength);
            ledStrip.setData(ledBuffer);
            rainbowHue %= 180;
        }
    }

    @Override
    public void periodic() {
        switch (mode){
            case SOLID:
                setSolidColor(currentColor);

            case BLINK:
                if (currentColor == primary) currentColor = secondary;
                else currentColor = primary;

                timer.advanceIfElapsed(blinkTime);
                setSolidColor(currentColor);
                timer.reset();

            case FADE:
                setSolidColor(fadeColor);

            case RAINBOW:
                setRainbow();
        }
    }
}
