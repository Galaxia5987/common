package frc.robot.leds;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

public class LedStrip extends SubsystemBase {
    private final AddressableLED ledStrip;
    private AddressableLEDBuffer ledBuffer;
    private LedState state = new LedState();

    private Color currentColor = state.getPrimary();
    private Color fadeColor = state.getPrimary();

    private final Timer timer = new Timer();

    public LedStrip(int port, int length) {
        ledStrip = new AddressableLED(port);
        ledBuffer = new AddressableLEDBuffer(length);

        ledStrip.setLength(length);
        ledStrip.setData(ledBuffer);
        ledStrip.start();

        timer.start();
        timer.reset();
    }

    public void setState(LedState state) {
        this.state = state;
    }

    private void setSolidColor(Color color, int start, int end) {
        for (int i = start; i < end; i++) {
            ledBuffer.setLED(i, color);
        }
        ledStrip.setData(ledBuffer);
    }

    private void setSolidColor(Color color) {
        setSolidColor(color, state.getStartingLed() - 1, state.getEndingLed());
    }

    /**
     * Updates fadeColor to the correct color for a fade effect at the current time by interpolating
     * the two given colors.
     *
     * @param initial Initial color.
     * @param goal Final Color.
     */
    private void updateFade(Color initial, Color goal) {
        Translation3d initialPoint = new Translation3d(initial.red, initial.green, initial.blue);
        Translation3d goalPoint = new Translation3d(goal.red, goal.green, goal.blue);

        double d = initialPoint.getDistance(goalPoint) / state.getFadeDuration();
        double t = d / (initialPoint.minus(goalPoint)).getNorm();

        Translation3d solution = initialPoint.interpolate(goalPoint, t);
        fadeColor = new Color((int) solution.getX(), (int) solution.getY(), (int) solution.getZ());
    }

    private void setRainbow() {
        int rainbowHue = 0;
        for (int i = state.getStartingLed() - 1; i < state.getEndingLed(); i++) {
            ledBuffer.setHSV(i, rainbowHue, 255, 180);
            rainbowHue += (180 / state.getStripLength());
            ledStrip.setData(ledBuffer);
            rainbowHue %= 180;
        }
    }

    public Command solid() {
        return this.run(() -> setSolidColor(state.getPrimary()));
    }

    public Command percentage(IntSupplier percentage) {
        return this.run(
                () -> {
                    state.setPercentage(percentage.getAsInt());
                    setSolidColor(
                            state.getPrimary(),
                            0,
                            state.getStripLength() * state.getPercentage() / 100);
                });
    }

    public Command percentage() {
        return percentage(() -> state.getPercentage());
    }

    public Command blink(DoubleSupplier blinkTime) {
        return this.run(
                () -> {
                    state.setBlinkTime(blinkTime.getAsDouble());
                    if (currentColor == state.getPrimary()) currentColor = state.getSecondary();
                    else currentColor = state.getPrimary();

                    if (timer.advanceIfElapsed(state.getBlinkTime()) && state.getBlinkTime() > 0) {
                        setSolidColor(currentColor);
                    }
                });
    }

    public Command blink() {
        return blink(() -> state.getBlinkTime());
    }

    public Command fade(DoubleSupplier fadeDuration) {
        return this.run(
                () -> {
                    state.setFadeDuration(fadeDuration.getAsDouble());
                    updateFade(state.getPrimary(), state.getSecondary());
                    setSolidColor(fadeColor);
                });
    }

    public Command fade() {
        return fade(() -> state.getFadeDuration());
    }

    public Command rainbow() {
        return this.run(this::setRainbow);
    }
}
