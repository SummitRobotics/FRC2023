// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIntake;
import frc.robot.subsystems.arm.ArmPositions;
import frc.robot.subsystems.arm.ArmIntake.INTAKE_ELEMENT_TYPE;
import frc.robot.subsystems.arm.ArmIntake.State;
import frc.robot.subsystems.arm.ArmPositions.ARM_POSITION;
import frc.robot.commands.arm.EjectElement;
import frc.robot.commands.arm.MoveArmUnsafe;
import frc.robot.commands.automovements.AutoPickup.ELEMENT_TYPE;
import frc.robot.commands.drivetrain.FollowPathPlannerTrajectory;
import frc.robot.subsystems.Drivetrain;
import frc.robot.commands.automovements.AutoPickup;
import frc.robot.commands.automovements.AutoPickup.LOCATION;
import frc.robot.commands.drivetrain.EncoderDrive;
import frc.robot.commands.drivetrain.TurnByEncoder;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OnePointFive extends SequentialCommandGroup {
  /** Creates a new twoPieceBetter. */
  public enum Type {
    FarFromSubstation,
    CloseToSubstation
  }

  public OnePointFive(Arm arm, ArmIntake armIntake, Drivetrain drivetrain, Type type, Alliance alliance) {
    PathPlannerTrajectory firstTraj;
        PathPlannerTrajectory secondTraj;
        ARM_POSITION placePos;
        ARM_POSITION grabPos;

        if (type == Type.FarFromSubstation) {
            firstTraj = PathPlanner.loadPath("FarFromSubstation", new PathConstraints(3, 3));
            secondTraj = PathPlanner.loadPath("FarFromSubstationBack", new PathConstraints(3, 3));
            if (alliance == Alliance.Red) {
                placePos = ARM_POSITION.AUTO_PLACE_LEFT;
                grabPos = ARM_POSITION.AUTO_GRAB_LEFT;
            } else {
                placePos = ARM_POSITION.AUTO_PLACE_RIGHT;
                grabPos = ARM_POSITION.AUTO_GRAB_RIGHT;
            }
        } else {
            if (alliance == Alliance.Red) {
                firstTraj = PathPlanner.loadPath("CloseToSubstation", new PathConstraints(3, 3));
                secondTraj = PathPlanner.loadPath("CloseToSubstationBack", new PathConstraints(3, 3));
                placePos = ARM_POSITION.AUTO_PLACE_RIGHT;
                grabPos = ARM_POSITION.AUTO_GRAB_RIGHT;
            } else {
                firstTraj = PathPlanner.loadPath("FarFromSubstation", new PathConstraints(3, 3));
                secondTraj = PathPlanner.loadPath("FarFromSubstationBack", new PathConstraints(3, 3));
                placePos = ARM_POSITION.AUTO_PLACE_LEFT;
                grabPos = ARM_POSITION.AUTO_GRAB_LEFT;
            }
        }

        addCommands(
          new InstantCommand(() -> armIntake.setState(State.STATIONARY)),
          new InstantCommand(() -> armIntake.setType(INTAKE_ELEMENT_TYPE.CONE)),
          new InstantCommand(() -> armIntake.setState(State.STALLING)),
          new InstantCommand(drivetrain::highGear),
          new ParallelCommandGroup(
            new EncoderDrive(-0.75, drivetrain),
            new ArmOutOfStart(arm)
          ),
          new MoveArmUnsafe(arm, ARM_POSITION.MIDDLE_HIGH),
          new EncoderDrive(0.75, drivetrain),
          new WaitCommand(1),
          new MoveArmUnsafe(arm, ARM_POSITION.MIDDLE_HIGH),
          new EjectElement(armIntake),
          new InstantCommand(() -> AutoPickup.setType(ELEMENT_TYPE.QUORB)),
          new ParallelCommandGroup(
            new SequentialCommandGroup(
              new WaitCommand(.5),
              new EncoderDrive(-4, drivetrain)
            ),
            new MoveArmUnsafe(arm, ARM_POSITION.HOME)
          ),
          new TurnByEncoder(270, drivetrain),
          new InstantCommand(armIntake::stop),
          
          new ParallelCommandGroup(
            new AutoPickup(arm, armIntake, drivetrain, LOCATION.GROUND),
            new SequentialCommandGroup(
              new WaitCommand(3),
              new EncoderDrive(.5, drivetrain)
            )
          )
            // new WaitCommand(0.25),
            // new EncoderDrive(-1, drivetrain),
            // new TurnByEncoder(-245, drivetrain),
            // new SequentialCommandGroup(
            //   new MoveArmUnsafe(arm, placePos),
            //   new EncoderDrive(4, drivetrain)
            // ),
            // new EjectElement(armIntake)
          

        );
    
  }
}


