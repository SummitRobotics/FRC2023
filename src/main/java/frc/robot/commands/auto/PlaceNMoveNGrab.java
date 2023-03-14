package frc.robot.commands.auto;

import org.photonvision.PhotonCamera;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.arm.MoveArmUnsafe;
import frc.robot.commands.automovements.AutoPickup;
import frc.robot.commands.drivetrain.EncoderDrive;
import frc.robot.commands.drivetrain.TurnByEncoder;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmPositions.ARM_POSITION;

public class PlaceNMoveNGrab extends SequentialCommandGroup {
    public PlaceNMoveNGrab(Arm arm, Drivetrain drivetrain, PhotonCamera quorbCamera, PhotonCamera coneCamera) {
        addCommands(
            new InstantCommand(drivetrain::highGear),
            new ParallelCommandGroup(
                new ArmOutOfStart(arm),
                new EncoderDrive(-0.75, drivetrain)
            ),
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    new MoveArmUnsafe(arm, ARM_POSITION.MIDDLE_HIGH),
                    new WaitCommand(0.25)
                ),
                new EncoderDrive(0.75, drivetrain)
            ),
            new InstantCommand(arm::unclamp),
            new EncoderDrive(-4.25, drivetrain),
            new TurnByEncoder(180, drivetrain),
            new AutoPickup(drivetrain, arm, quorbCamera, coneCamera)
        );
    }
}
