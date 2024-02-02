package frc.robot.leds;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LedStrip extends SubsystemBase {
    private final AddressableLED ledStrip;
    private AddressableLEDBuffer ledBuffer;
    private int stripLength = 0;
    private int offset = 0;
    private int percentage = 100;

    private double blinkTime = 1;
    private double fadeDuration = 1;

    private Color primary = new Color();
    private Color secondary = new Color();
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
     * Sets the offset of the LEDs.
     * @param offset The amount of LEDs that won't light.
     *               Counts from the start of the strip.
     */
    public void setOffset(int offset){
        setSolidColor(Color.kBlack, 0, offset+1);
        this.offset = offset;
    }

    public void setPercentage(int percentage) {
        this.percentage = percentage;
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
        var appliedColor = new Color(color.red, color.green, color.blue);
        for (int i = offset; i < stripLength; i++) {
            ledBuffer.setLED(i, appliedColor);
        }
        ledStrip.setData(ledBuffer);
    }

    private void setSolidColor(Color color, int start, int end) {
        var appliedColor = new Color(color.red, color.green, color.blue);
        for (int i = start; i < end; i++) {
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
    private void updateFade(Color initial, Color goal) {
        Translation3d initialPoint = new Translation3d(initial.red, initial.green, initial.blue);
        Translation3d goalPoint = new Translation3d(goal.red, goal.green, goal.blue);

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
        for (int i = offset; i < stripLength; i++) {
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
                setSolidColor(primary);

            case PERCENTAGE:
                setSolidColor(primary, 0, stripLength*percentage/100);

            case BLINK:
                if (currentColor == primary) currentColor = secondary;
                else currentColor = primary;

                if (timer.advanceIfElapsed(blinkTime)) {
                    setSolidColor(currentColor);
                }
                timer.reset();

            case FADE:
                updateFade(primary, secondary);
                setSolidColor(fadeColor);

            case RAINBOW:
                setRainbow();
        }
    }
}
