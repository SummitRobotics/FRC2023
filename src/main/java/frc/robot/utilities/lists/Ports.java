// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities.lists;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Ports {

    public static final class OI {
        public static final int
            XBOX_PORT = 0,
            LAUNCHPAD_PORT = 1,
            JOYSTICK_PORT = 2;
    }

    public static final class Drivetrain {
        public static final int
            LEFT_1 = 10,
            LEFT_2 = 11,
            LEFT_3 = 12,
            RIGHT_1 = 13,
            RIGHT_2 = 14,
            RIGHT_3 = 15,
            SHIFT_SOLENOID = 4;
    }

    public static final class LED {
        public static final int
            PORT = 0,
            LENGTH = 114;
    }

    public static final class Other {
        public static final int
        PRESSURE_SENSOR = 0,
        PCM = 2,
        PDP = 1;
    }
}
