/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utilities;

import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.function.BooleanSupplier;

/**
 * takes a button or boolean supplier and makes a new button.
 * Which is only true on the rising edge of the press
 */
public class SimpleTrigger extends Trigger {

    private final BooleanSupplier supplier;
    private boolean last;

    /**
     * Constructor to create a SimpleButton.
     *
     * @param supplier BooleanSupplier
     */
    public SimpleTrigger(BooleanSupplier supplier) {
        super(() -> false);
        this.supplier = supplier;
        last = false;
    }

    /**
     * Creates a Simple Button from a Button.
     *
     * @param supplier A Button
     */
    public SimpleTrigger(Trigger supplier) {
        super(() -> false);
        this.supplier = supplier::getAsBoolean;
        last = false;
    }

    /**
     * Returns true only on the rising edge of a button press.
     *
     * @return only true on the rising edge.
     */
    public boolean get() {
        boolean current = supplier.getAsBoolean();
        boolean output = !last && current;

        last = current;

        return output;
    }
}
