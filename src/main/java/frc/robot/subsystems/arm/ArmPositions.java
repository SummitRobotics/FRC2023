package frc.robot.subsystems.arm;

public class ArmPositions {
    public static enum ARM_POSITION {
        MIDDLE_LOW(31.59497,98.004,28.547,-23.92842,-46.7615),
        MIDDLE_MEDIUM(31.59497,63.2326,81.40302, -63.9060,-46.7615),
        MIDDLE_HIGH(31.59497,140.5,140,-52,-46.7615),

        LEFT_LOW(34.9282/3,98.004,28.547,-23.92842,-46.7615),
        LEFT_MEDIUM(34.9282/3,63.2326,81.40302, -63.9060,-46.7615),
        LEFT_HIGH(34.9282/3,140.5,140,-52,-46.7615),

        RIGHT_LOW(150.71/3,98.004,28.547,-23.92842,-46.7615),
        RIGHT_MEDIUM(155.471/3,63.2326,81.40302, -63.9060,-46.7615),
        RIGHT_HIGH(155.471/3,140.5,140,-52,-46.7615),

        HOME(31.45211, 39.21389, 1.40476, -23.40462, -46.73772),
        HIGH_ASPECT(32.0711, 36.8805, 93.622, -77.502, -41.09),
        STARTING_CONFIG(31.8330, 44.8804, 22.7617, -158.8284, -52.2382),
        PRE_HOME(10,10,10,-10,-10),
        AUTO_DROP_OFF(52.3565,114.7627,137.0931,-92.4323,-46.7615);
            
        public ArmConfiguration config;
        ARM_POSITION(double turret, double joint1, double joint2, double joint3, double wrist) {
          config = new ArmConfiguration(turret, joint1, joint2, joint3, wrist, ArmConfiguration.POSITION_TYPE.ENCODER_ROTATIONS);
        }
      }
}
