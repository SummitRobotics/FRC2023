package frc.robot.subsystems.arm;

public class ArmPositions {
    public static enum ARM_POSITION {
        MIDDLE_LOW(31.59497,98.004,28.547,-23.92842),
        MIDDLE_MEDIUM(31.59497,63.2326,81.40302, -63.9060),
        MIDDLE_HIGH(31.59497,141.99,139.14054,-56.71488),

        LEFT_LOW(34.9282/3,98.004,28.547,-23.92842),
        LEFT_MEDIUM(34.9282/3,63.2326,81.40302, -63.9060),
        LEFT_HIGH(34.9282/3,93.027,128.451,-101.4549),
        AUTO_PLACE_FAR_BLUE(34.9282/3,93.027,128.451,-86.4549),

        RIGHT_LOW(150.71/3,98.004,28.547,-23.92842),
        RIGHT_MEDIUM(155.471/3,63.2326,81.40302, -63.9060),
        RIGHT_HIGH(155.471/3,93.027,128.451,-101.4549),

        HOME(31.45211, 39.21389, 1.40476, -23.40462),
        HIGH_ASPECT(32.0711, 36.8805, 93.622, -77.502),
        STARTING_CONFIG(31.8330, 44.8804, 22.7617, -158.8284),
        PRE_HOME(15,15,15,-15),
        AUTO_DROP_OFF(52.3565,114.7627,137.0931,-92.4323),

        GROUND_PICKUP_SAFE(31.45211,39.21389,13.357,-23.40462),
        GROUND_PICKUP_CONE(31.45211,95.408,13.357,-45.904),
        GROUND_PICKUP_QUORB(31.45211,95.408,21.904,-56.143),
        AUTO_GRAB_FAR_BLUE(22.881, 131.570, 79.788, -112.572);
            
        public ArmConfiguration config;
        ARM_POSITION(double turret, double joint1, double joint2, double joint3) {
          config = new ArmConfiguration(turret, joint1, joint2, joint3, ArmConfiguration.POSITION_TYPE.ENCODER_ROTATIONS);
        }
      }
}
