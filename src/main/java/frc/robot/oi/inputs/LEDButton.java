package frc.robot.oi.inputs;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import java.util.function.BooleanSupplier;

/**
 * Class for LED Buttons.
 */
public class LEDButton extends OITrigger {

    /**
     * LED interface.
     */
    public interface LED {
        void set(boolean state);
    }

    protected Command controller;

    private LED led;

    /**
     * Constructor to create a LED Button.
     *
     * @param getter a boolean supplier.
     * @param led the LED for the button.
     */
    public LEDButton(BooleanSupplier getter, LED led) {
        super(getter);
        this.led = led;

        controller = new StartEndCommand(() -> led.set(true), () -> led.set(false));
    }

    public LEDButton() {
        super();
    }

    public void setLED(boolean on){
        led.set(on);
    }
}
