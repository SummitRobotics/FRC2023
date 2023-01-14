package frc.robot.oi.drivers;

import frc.robot.oi.inputs.OIAxis;
import frc.robot.oi.inputs.OITrigger;
import frc.robot.utilities.Functions;

/**
 * Wrapper class for basic joystick functionality.
 */
public class JoystickDriver extends GenericDriver {

    public OITrigger button5, button2, button3, button4, button6, 
        trigger, button7, button8, button9, button10, button11;
    public OIAxis axisX, axisY, axisZ;
    private boolean assureZUp = false, assureZDown = false;

    /**
     * Constructor to make a basic joystick driver.
     *
     * @param port The port of the joystick. This is retrieved from the driver station
     */
    public JoystickDriver(int port) {
        super(port);

        trigger = generateOITrigger(1);

        button3 = generateOITrigger(3);
        button4 = generateOITrigger(4);
        button5 = generateOITrigger(5);
        button2 = generateOITrigger(2);
        button6 = generateOITrigger(6);
        button7 = generateOITrigger(7);
        button8 = generateOITrigger(8);
        button9 = generateOITrigger(9);
        button10 = generateOITrigger(10);
        button11 = generateOITrigger(11);
        axisX = generateOIAxis(0);
        axisY = generateOIAxis(1);
        // axisZ = generateOIAxis(2);

        axisZ =
                new OIAxis(getAxisGetter(2)) {
                    @Override
                    public double get() {
                        double position = (getter.getAsDouble() - 1) / -2;

                        if (!assureZUp) {
                            if (position > .95) {
                                assureZUp = true;
                                // System.out.println("axis z is up");
                            }

                            return 0;
                        }

                        if (!assureZDown) {
                            if (position < .05) {
                                assureZDown = true;
                                // System.out.println("axis z is down, activated");
                            }

                            return 0;
                        }

                        if (Functions.isWithin(position, 0, deadzone)) {
                            return 0;
                        }

                        return (1 + deadzone) * position - Math.copySign(deadzone, position);
                    }
                };
    }

    public void reEnableJoystickCalibrationCheck() {
        assureZDown = false;
        assureZUp = false;
    }
}
