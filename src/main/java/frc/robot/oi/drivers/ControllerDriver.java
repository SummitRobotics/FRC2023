package frc.robot.oi.drivers;

import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.oi.inputs.OIAxis;
import frc.robot.oi.inputs.OITrigger;
import java.util.function.BooleanSupplier;

/**
 * Wrapper class for XBox controllers.
 */
public class ControllerDriver extends GenericDriver {

    /**
     * Enum for the possible DPAD values and positions.
     */
    public enum DPadValues {
        UP(0, 45, 315),
        DOWN(180, 135, 225),
        RIGHT(90, 45, 135),
        LEFT(270, 225, 315);

        public final int[] values;

        DPadValues(int... values) {
            this.values = values;
        }

        /**
         * Checks to see if a value is equal to a DPadValue.
         *
         * @param value A value to compare
         * @return a boolean weather it is equal or not
         */
        public boolean isEqual(int value) {
            for (int element : values) {
                if (value == element) {
                    return true;
                }
            }
            return false;
        }
    }

    public OITrigger
            buttonA,
            buttonB,
            buttonX,
            buttonY,
            buttonStart,
            buttonBack,
            rightBumper,
            leftBumper,
            dPadUp,
            dPadDown,
            dPadLeft,
            dPadRight,
            dPadAny;
            
    public OIAxis leftX, leftY, leftTrigger, rightTrigger, rightX, rightY;

    /**
     * Constructor for generating a Controller Driver.
     *
     * @param port the port of the control device
     *             this is found on the driver station.
     */
    public ControllerDriver(int port) {
        super(port);
        buttonA = generateOITrigger(Button.kA.value);
        buttonB = generateOITrigger(Button.kB.value);
        buttonX = generateOITrigger(Button.kX.value);
        buttonY = generateOITrigger(Button.kY.value);
        buttonStart = generateOITrigger(Button.kStart.value);
        buttonBack = generateOITrigger(Button.kBack.value);
        rightBumper = generateOITrigger(Button.kRightBumper.value);
        leftBumper = generateOITrigger(Button.kLeftBumper.value);

        dPadUp = new OITrigger(getDPadValue(DPadValues.UP));
        dPadDown = new OITrigger(getDPadValue(DPadValues.DOWN));
        dPadLeft = new OITrigger(getDPadValue(DPadValues.LEFT));
        dPadRight = new OITrigger(getDPadValue(DPadValues.RIGHT));
        dPadAny = new OITrigger(() -> getPOV() != -1);

        leftX = generateOIAxis(0);
        leftY = generateOIAxis(1);
        leftTrigger = generateOIAxis(2);
        rightTrigger = generateOIAxis(3);
        rightX = generateOIAxis(4);
        rightY = generateOIAxis(5);
    }

    private BooleanSupplier getDPadValue(DPadValues value) {
        return () -> value.isEqual(getPOV());
    }
}