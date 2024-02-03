package frc.robot.leds;

import edu.wpi.first.wpilibj.util.Color;

public class LedState {
    private LedMode mode;
    private Color primary;
    private Color secondary;
    private double blinkTime;
    private double fadeDuration;
    private int percentage;
    private int startingLed;
    private int endingLed;

    public LedState(LedMode mode, Color primary, Color secondary, double blinkTime, double fadeDuration) {
        this.mode = mode;
        this.primary = primary;
        this.secondary = secondary;
        this.blinkTime = blinkTime;
        this.fadeDuration = fadeDuration;
    }

    public LedState(){
        this.mode = LedMode.SOLID;
        this.primary = Color.kBlack;
        this.secondary = Color.kBlack;
        this.blinkTime = 1;
        this.fadeDuration = 1;
        this.percentage = 100;
        this.startingLed = 1;
        this.endingLed = 1;
    }

    /**
     * @return Active length of the strip.
     */
    public int getStripLength(){
        return endingLed-startingLed+1;
    }

    public void setStripLength(int length){
        setEndingLed(startingLed-1+length);
    }

    public void setStripLength(int startingLed, int endingLed){
        setStartingLed(startingLed);
        setEndingLed(endingLed);
    }

    public LedMode getMode() {
        return mode;
    }

    public void setMode(LedMode mode) {
        this.mode = mode;
    }

    public Color getPrimary() {
        return primary;
    }

    /**
     * Sets the primary color of the LEDs that will be used
     * in the solid and percentage modes and as the first color
     * in the blink and fade modes.
     * @param primary Color to set the primary.
     */
    public void setPrimary(Color primary) {
        this.primary = primary;
    }

    public Color getSecondary() {
        return secondary;
    }

    /**
     * Sets the secondary color of the LEDs that will be used
     * as the second color in the blink and fade modes.
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
     * @param time Time it takes to change the color. [sec]
     */
    public void setBlinkTime(double time) {
        this.blinkTime = time;
    }

    public double getFadeDuration() {
        return fadeDuration;
    }

    /**
     * Sets the duration for the fade effect.
     * @param duration Duration of the fade effect. [sec]
     */
    public void setFadeDuration(double duration) {
        this.fadeDuration = duration;
    }

    public int getPercentage() {
        return percentage;
    }

    /**
     * Sets the percentage of the active strip length to use.
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
}
