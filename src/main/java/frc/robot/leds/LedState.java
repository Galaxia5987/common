package frc.robot.leds;

import java.awt.*;

public class LedState {
    private LedMode mode;
    private Color primary;
    private Color secondary;
    private double blinkTime;
    private double fadeDuration;

    public LedState(LedMode mode, Color primary, Color secondary, double blinkTime, double fadeDuration) {
        this.mode = mode;
        this.primary = primary;
        this.secondary = secondary;
        this.blinkTime = blinkTime;
        this.fadeDuration = fadeDuration;
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

    public void setPrimary(Color primary) {
        this.primary = primary;
    }

    public Color getSecondary() {
        return secondary;
    }

    public void setSecondary(Color secondary) {
        this.secondary = secondary;
    }

    public double getBlinkTime() {
        return blinkTime;
    }

    public void setBlinkTime(double time) {
        this.blinkTime = time;
    }

    public double getFadeDuration() {
        return fadeDuration;
    }

    public void setFadeDuration(double duration) {
        this.fadeDuration = duration;
    }
}
