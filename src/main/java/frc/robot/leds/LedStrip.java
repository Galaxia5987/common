package frc.robot.leds;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.IntSupplier;

public class LedStrip extends SubsystemBase {
    private final AddressableLED ledStrip;
    private AddressableLEDBuffer ledBuffer;
    private int stripLength = 1;
    private int startingLed = 1;
    private int endingLed = 1;

    private Color primary = Color.kBlack;
    private Color secondary = Color.kBlack;
    private Color currentColor = primary;
    private Color fadeColor = primary;

    private double blinkTime = 1;
    private double fadeTime = 1;
    private int percentage = 100;

    private final Timer timer = new Timer();

    public LedStrip(int port, int length) {
        ledStrip = new AddressableLED(port);
        ledBuffer = new AddressableLEDBuffer(length);
        setEndingLed(length);

        ledStrip.setLength(length);
        ledStrip.setData(ledBuffer);
        ledStrip.start();

        timer.start();
        timer.reset();
    }

    private void setSolidColor(Color color, int start, int end) {
        for (int i = start; i < end; i++) {
            ledBuffer.setLED(i, color);
        }
        ledStrip.setData(ledBuffer);
    }

    private void setSolidColor(Color color) {
        setSolidColor(color, startingLed - 1, endingLed);
    }

    /**
     * Updates fadeColor to the correct color for a fade effect at the current time by interpolating
     * the two given colors.
     *
     * @param initial Initial color.
     * @param goal    Final Color.
     */
    private void updateFade(Color initial, Color goal) {
        Translation3d initialPoint = new Translation3d(initial.red, initial.green, initial.blue);
        Translation3d goalPoint = new Translation3d(goal.red, goal.green, goal.blue);

        double d = initialPoint.getDistance(goalPoint) / fadeTime;
        double t = d / (initialPoint.minus(goalPoint)).getNorm();

        Translation3d solution = initialPoint.interpolate(goalPoint, t);
        fadeColor = new Color((int) solution.getX(), (int) solution.getY(), (int) solution.getZ());
    }

    private void setRainbow() {
        int rainbowHue = 0;
        for (int i = startingLed - 1; i < endingLed; i++) {
            ledBuffer.setHSV(i, rainbowHue, 255, 180);
            rainbowHue += (180 / stripLength);
            ledStrip.setData(ledBuffer);
            rainbowHue %= 180;
        }
    }

    /**
     * @return Active length of the strip.
     */
    public int getStripLength() {
        return endingLed - startingLed + 1;
    }

    public void setStripLength(int length) {
        setEndingLed(startingLed - 1 + length);
    }

    public void setStripLength(int startingLed, int endingLed) {
        setStartingLed(startingLed);
        setEndingLed(endingLed);
    }

    public Color getPrimary() {
        return primary;
    }

    /**
     * Sets the primary color of the LEDs that will be used in the solid and percentage modes and as
     * the first color in the blink and fade modes.
     *
     * @param primary Color to set the primary.
     */
    public void setPrimary(Color primary) {
        this.primary = primary;
    }

    public Color getSecondary() {
        return secondary;
    }

    /**
     * Sets the secondary color of the LEDs that will be used as the second color in the blink and
     * fade modes.
     *
     * @param secondary Color to set the secondary.
     */
    public void setSecondary(Color secondary) {
        this.secondary = secondary;
    }

    public double getBlinkTime() {
        return blinkTime;
    }

    /**
     * Sets the blink time of the LEDs.
     *
     * @param time Time it takes to change the color. [sec]
     */
    public void setBlinkTime(double time) {
        this.blinkTime = time;
    }

    public double getFadeTime() {
        return fadeTime;
    }

    /**
     * Sets the duration for the fade effect.
     *
     * @param duration Duration of the fade effect. [sec]
     */
    public void setFadeTime(double duration) {
        this.fadeTime = duration;
    }

    public int getPercentage() {
        return percentage;
    }

    /**
     * Sets the percentage of the active strip length to use.
     *
     * @param percentage Percent of active strip length. [%]
     */
    public void setPercentage(int percentage) {
        this.percentage = percentage;
    }

    public int getStartingLed() {
        return startingLed;
    }

    public void setStartingLed(int startingLed) {
        this.startingLed = startingLed;
    }

    public int getEndingLed() {
        return endingLed;
    }

    public void setEndingLed(int endingLed) {
        this.endingLed = endingLed;
    }

    /**
     * Sets a solid color that won't be saved as the primary color.
     *
     * @param optionalColor A color to set the LEDs to.
     * @return A command that sets the LEDs to a solid color.
     */
    public Command solid(Color optionalColor) {
        return this.run(() -> setSolidColor(optionalColor));
    }

    public Command solid(){
        return solid(primary);
    }

    public Command percentage(IntSupplier percentage) {
        return this.run(
                () -> {
                    setPercentage(percentage.getAsInt());
                    setSolidColor(
                            primary,
                            0,
                            stripLength * this.percentage / 100);
                });
    }

    public Command percentage() {
        return percentage(() -> percentage);
    }

    public Command blink(double blinkTime) {
        return this.run(
                () -> {
                    setBlinkTime(blinkTime);
                    currentColor = currentColor == primary ?
                            secondary : primary;

                    if (timer.advanceIfElapsed(this.blinkTime) && this.blinkTime > LedConstants.MINIMAL_BLINK_TIME) {
                        setSolidColor(currentColor);
                    }
                });
    }

    public Command blink() {
        return blink(blinkTime);
    }

    public Command fade(double fadeTime) {
        return this.run(
                () -> {
                    setFadeTime(fadeTime);
                    updateFade(primary, secondary);
                    setSolidColor(fadeColor);
                });
    }

    public Command fade() {
        return fade(fadeTime);
    }

    public Command rainbow() {
        return this.run(this::setRainbow);
    }
}
