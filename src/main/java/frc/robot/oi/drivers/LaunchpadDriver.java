package frc.robot.oi.drivers;

import frc.robot.oi.inputs.LEDButton;
import frc.robot.oi.inputs.LEDButton.LED;
import frc.robot.oi.inputs.OIAxis;
import frc.robot.oi.inputs.OITrigger;
import java.util.function.DoubleSupplier;

import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.simulation.DriverStationDataJNI;


/**
 * Wrapper class for the TI Launchpad in mode 1.
 */
public class LaunchpadDriver extends GenericDriver {

    private int outputs;

    public LEDButton
            buttonA, buttonB, buttonC, buttonD, buttonE, buttonF, buttonG, buttonH, buttonI;
    public OITrigger missileA, missileB, funLeft, funMiddle, funRight;
    public OIAxis axisA, axisB, axisC, axisD, axisE, axisF, axisG, axisH;

    public DoubleSupplier reee;

    public LED bigLEDGreen, bigLEDRed;

    /**
     * Constructor for the TI Launchpad.\
     *
     * @param port The port of the TI Launchpad. This is retrieved from the driver station.
     */
    public LaunchpadDriver(int port) {
        super(port);

        buttonA = generateLEDButton(1);
        buttonB = generateLEDButton(2);
        buttonC = generateLEDButton(3);
        buttonD = generateLEDButton(4);
        buttonE = generateLEDButton(5);
        buttonF = generateLEDButton(6);
        buttonG = generateLEDButton(7);
        buttonH = generateLEDButton(8);
        buttonI = generateLEDButton(9);

        missileA = generateOITrigger(10);
        missileB = generateOITrigger(11);

        bigLEDGreen = getLEDLambda(10);
        bigLEDRed = getLEDLambda(11);

        axisA = generateOIAxis(0);
        axisB = generateOIAxis(1);

        axisA.setDeadzone(0);
        axisB.setDeadzone(0);

        reee = getAxisGetter(2);

        funLeft = generateATDRangeButton(2, -1.0, -0.95);
        funMiddle = generateATDRangeButton(2, -0.03, 0.03);
        funRight = generateATDRangeButton(2, 0.95, 1.0);

        axisC = generateOIAxis(3);
        axisD = generateOIAxis(4);
        axisE = generateOIAxis(5);
        axisF = generateOIAxis(6);
        axisG = generateOIAxis(7);
        axisH = generateOIAxis(8);
    }

    protected LEDButton generateLEDButton(int output) {
        return new LEDButton(getButtonGetter(output), (boolean state) -> setOutput(output, state));
    }

    private LED getLEDLambda(int output) {
        return (boolean state) -> setOutput(output, state);
    }

    private OITrigger generateATDRangeButton(int output, double min, double max) {
        return new OITrigger(() -> {
            double value = getAxisGetter(output).getAsDouble();
            return value >= min && max >= value;
        });
    }

    /**
     * Black box to set outputs copied from wpilib.
     *
     * @param outputNumber the output number
     * @param value        the state of the output
     */
    public void setOutput(int outputNumber, boolean value) {
        outputs = (outputs & ~(1 << (outputNumber - 1))) | ((value ? 1 : 0) << (outputNumber - 1));
        DriverStationJNI.setJoystickOutputs((byte) port, outputs, (short) 0, (short) 0);
    }
}
