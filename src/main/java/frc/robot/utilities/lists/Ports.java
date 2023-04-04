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
            DRIVER_XBOX_PORT = 0,
            GUNNER_XBOX_PORT = 2,
            LAUNCHPAD_PORT = 1,
            DA_BUTTON_PORT = 5;
            }

    public static final class Drivetrain {
        public static final int
            LEFT_1 = 11,
            LEFT_2 = 12,
            LEFT_3 = 13,
            RIGHT_1 = 14,
            RIGHT_2 = 15,
            RIGHT_3 = 16,
            SHIFT_SOLENOID = 0;
    }

    public static final class Arm {
        public static final int
            TURRET = 20,
            JOINT_1 = 21,
            JOINT_2 = 22,
            JOINT_3 = 23,
            WRIST = 24,
            CLAMP_SOLENOID = 1,
            // TODO - add port for intake motor
            INTAKE_MOTOR = 24;
    }
    
    public static final class Intake {
        public static final int
            INTAKE_MOTOR = 30,
            PIVOT_MOTOR = 31,
            LOCK_SOLENOID = 0;
    }
    
    public static final class LED {
        public static final int
            PORT = 0,
            LENGTH = 60;
    }

    public static final class Other {
        public static final int
        PRESSURE_SENSOR = 0,
        PCM = 2,
        PDP = 1;
    }
}
