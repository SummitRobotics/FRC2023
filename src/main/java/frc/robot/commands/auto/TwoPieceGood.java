package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.arm.EjectElement;
import frc.robot.commands.arm.MoveArmUnsafe;
import frc.robot.commands.automovements.AutoPickup;
import frc.robot.commands.automovements.AutoPickup.LOCATION;
import frc.robot.commands.drivetrain.ChargeBalance;
import frc.robot.commands.drivetrain.EncoderDrive;
import frc.robot.commands.drivetrain.TurnByEncoder;
import frc.robot.commands.drivetrain.ChargeBalance.BalanceDirection;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIntake;
import frc.robot.subsystems.arm.ArmIntake.INTAKE_ELEMENT_TYPE;
import frc.robot.subsystems.arm.ArmIntake.State;
import frc.robot.subsystems.arm.ArmPositions.ARM_POSITION;

public class TwoPieceGood extends SequentialCommandGroup {
  
    public TwoPieceGood(Arm arm, ArmIntake armIntake, Drivetrain drivetrain, Alliance alliance) {
      // boolean isLeft;
      // if (alliance == Alliance.Red) {
        // isLeft = true;
      // } else {
        // isLeft = false;
      // }
      addCommands(
          new InstantCommand(() -> armIntake.setState(State.STATIONARY)),
          new InstantCommand(() -> armIntake.setType(INTAKE_ELEMENT_TYPE.QUORB)),
          new InstantCommand(() -> armIntake.setState(State.STALLING)),
          new InstantCommand(drivetrain::highGear),
          new ArmOutOfStart(arm),
          // 0.4748 is the distance from charge station to nodes minus our bumper length
          // new ParallelCommandGroup(
              // new EncoderDrive(0.4748, drivetrain),
              // new MoveArmUnsafe(arm, ARM_POSITION.MIDDLE_HIGH)
          // ),
          new EjectElement(armIntake),
          new InstantCommand(drivetrain::lowGear),
          // new ParallelCommandGroup(
              // new SequentialCommandGroup(
                  // new WaitCommand(0.25),
                  // new MoveArmUnsafe(arm, ARM_POSITION.HOME)
              // ),
          new MoveArmUnsafe(arm, ARM_POSITION.HOME),
          new ParallelRaceGroup(
              new InstantCommand(() -> drivetrain.setBothMotorPower(-0.5), drivetrain).repeatedly(),
              new SequentialCommandGroup(
                  new WaitUntilCommand(() -> Math.abs(drivetrain.gyro.getRoll()) > 8),
                  new WaitCommand(0.5),
                  new WaitUntilCommand(() -> Math.abs(drivetrain.gyro.getRoll()) < 8),
                  new WaitUntilCommand(() -> Math.abs(drivetrain.gyro.getRoll()) > 10)
              )
          ),
          new InstantCommand(() -> drivetrain.setBothMotorPower(0), drivetrain),
          new EncoderDrive(-3, drivetrain),
          new ParallelCommandGroup(
            new AutoPickup(arm, armIntake, true, LOCATION.GROUND),
            new SequentialCommandGroup(
              new WaitCommand(3),
              new TurnByEncoder(-12.5, drivetrain),
              new EncoderDrive(0.5, drivetrain)
            )
          ),
          new MoveArmUnsafe(arm, ARM_POSITION.HOME),
          new ParallelRaceGroup(
              new InstantCommand(() -> drivetrain.setBothMotorPower(0.5), drivetrain).repeatedly(),
              new SequentialCommandGroup(
                  new WaitUntilCommand(() -> Math.abs(drivetrain.gyro.getRoll()) > 8),
                  new WaitCommand(0.5),
                  new WaitUntilCommand(() -> Math.abs(drivetrain.gyro.getRoll()) < 8),
                  new WaitUntilCommand(() -> Math.abs(drivetrain.gyro.getRoll()) > 10)
              )
          ),
          new InstantCommand(() -> drivetrain.setBothMotorPower(0))
          // new MoveArmUnsafe(arm, ARM_POSITION.MIDDLE_HIGH),
          // new EncoderDrive(.4748, drivetrain),
          // new EjectElement(armIntake),
          // new ParallelCommandGroup(
            // new EncoderDrive(-.4748, drivetrain),
            // new MoveArmUnsafe(arm, ARM_POSITION.HOME)
          // ),
          // new EncoderDrive(-.4748, drivetrain),
          // new ChargeBalance(drivetrain, BalanceDirection.BACKWARD)

      );
    }
}
