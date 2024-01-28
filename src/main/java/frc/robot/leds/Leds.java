package frc.robot.leds;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Leds extends SubsystemBase {
    public final LedStrip[] strips;

    public Leds(LedStrip... strips) {
        this.strips = strips;
    }

    public void setLength(int length, int stripIndex) {
        strips[stripIndex].setLength(length);
    }

    public void setLength(int length) {
        for (int i = 0; i < strips.length; i++) {
            strips[i].setLength(length);
        }
    }

    public void blink(int stripIndex) {
        strips[stripIndex].blink();
    }

    public void blink() {
        for (int i = 0; i < strips.length; i++) {
            strips[i].blink();
        }
    }

    public void setBlinkTime(double time, int stripIndex) {
        strips[stripIndex].setBlinkTime(time);
    }

    public void setBlinkTime(double time) {
        for (int i = 0; i < strips.length; i++) {
            strips[i].setBlinkTime(time);
        }
    }

    public void fade(int stripIndex) {
        strips[stripIndex].fade();
    }

    public void fade() {
        for (int i = 0; i < strips.length; i++) {
            strips[i].fade();
        }
    }

    public void setFadeDuration(double duration, int stripIndex) {
        strips[stripIndex].setFadeDuration(duration);
    }

    public void setFadeDuration(double duration) {
        for (int i = 0; i < strips.length; i++) {
            strips[i].setFadeDuration(duration);
        }
    }

    public void rainbow(int stripIndex) {
        strips[stripIndex].rainbow();
    }

    public void rainbow() {
        for (int i = 0; i < strips.length; i++) {
            strips[i].rainbow();
        }
    }

    @Override
    public void periodic() {
        for (int i = 0; i < strips.length; i++) {
            strips[i].updateStrip();
        }
    }
}
