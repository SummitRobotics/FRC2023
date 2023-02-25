// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.arm.ArmMO;
import frc.robot.commands.arm.FullManualArm;
import frc.robot.commands.arm.MoveArmHome;
import frc.robot.commands.arm.MoveArmUnsafe;
import frc.robot.commands.drivetrain.ArcadeDrive;
import frc.robot.oi.drivers.ControllerDriver;
import frc.robot.oi.drivers.LaunchpadDriver;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConfiguration;
import frc.robot.subsystems.arm.MovementMap;
import frc.robot.subsystems.arm.ArmConfiguration.POSITION_TYPE;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utilities.Positions;
import frc.robot.utilities.lists.AxisPriorities;
import frc.robot.utilities.lists.Ports;
import frc.robot.commands.Home;
import frc.robot.commands.LogComponents;
import frc.robot.commands.TimedMoveMotor;

public class RobotContainer {

    // Overall
    private CommandScheduler scheduler;
    private AHRS navx;

    // OI
    private ControllerDriver driverXBox;
    private ControllerDriver gunnerXBox;
    private LaunchpadDriver launchpad;

    // Subsystems
    private Drivetrain drivetrain;
    private Arm arm;

    // Commands
    private Command arcadeDrive;

    private Command turretManual;
    private Command joint1Manual;
    private Command joint2Manual;
    private Command joint3Manual;
    private Command wristManual;
    private Command fancyArmMo;

    private Command homeArm;

    private Command testCommand;

    public RobotContainer() {
        scheduler = CommandScheduler.getInstance();

        // OI
        driverXBox = new ControllerDriver(Ports.OI.DRIVER_XBOX_PORT);
        gunnerXBox = new ControllerDriver(Ports.OI.GUNNER_XBOX_PORT);
        launchpad = new LaunchpadDriver(Ports.OI.LAUNCHPAD_PORT);

        // Devices
        navx = new AHRS();

        // Subsystems
        drivetrain = Drivetrain.init(navx, new Pose2d());
        arm = new Arm();

        createCommands();
        setDefaultCommands();
        configureBindings();

        // Init Logging and Telemetry
        initLogging();
        initTelemetry();
    }

    private void createCommands() {
        arcadeDrive = new ArcadeDrive(drivetrain, driverXBox.leftTrigger, driverXBox.rightTrigger, driverXBox.leftX, driverXBox.dPadAny);

        turretManual = new FullManualArm(arm, FullManualArm.Type.TURRET, gunnerXBox);
        joint1Manual = new FullManualArm(arm, FullManualArm.Type.JOINT_1, gunnerXBox);
        joint2Manual = new FullManualArm(arm, FullManualArm.Type.JOINT_2, gunnerXBox);
        joint3Manual = new FullManualArm(arm, FullManualArm.Type.JOINT_3, gunnerXBox);
        wristManual = new FullManualArm(arm, FullManualArm.Type.WRIST, gunnerXBox);
        fancyArmMo = new ArmMO(arm, gunnerXBox);

        homeArm = new SequentialCommandGroup(new Home(arm.getHomeables()[4]), new Home(arm.getHomeables()[3]),
                        new ParallelCommandGroup(new TimedMoveMotor(arm::setWristMotorVoltage, -12, 0.25), new TimedMoveMotor(arm::setJoint3MotorVoltage, -3, 0.25)),
                        new Home(arm.getHomeables()[2], arm.getHomeables()[1]),
                        new ParallelCommandGroup(new TimedMoveMotor(arm::setJoint1MotorVoltage, 2, 0.2), new TimedMoveMotor(arm::setJoint2MotorVoltage, 2, 0.2)), new Home(arm.getHomeables()[0]),
                        new TimedMoveMotor(arm::setTurretMotorVoltage, 5, 0.1), new MoveArmHome(arm));

        // testCommand = new StartEndCommand(() -> {
        // double pos1 = 20;
        // double pos2 = 80;
        // double pos3 = -45;
        // double pos4 = -58;
        // double pos0 = 98;
        // ArmConfiguration config = new ArmConfiguration(pos0, pos1, pos2, pos3, pos4,
        // POSITION_TYPE.ENCODER_ROTATIONS);
        // double ff1 = Arm.joint1FF.calculate(pos1, config.getJoint1FFData());
        // double ff2 = Arm.joint2FF.calculate(pos2, config.getJoint2FFData());
        // double ff3 = Arm.joint3FF.calculate(pos3, config.getJoint3FFData());
        // arm.setTurretMotorRotations(pos0);
        // arm.setFirstJointMotorRotations(pos1, ff1);
        // arm.setSecondJointMotorRotations(pos2, ff2);
        // arm.setThirdJointMotorRotations(pos3, ff3);
        // arm.setWristMotorRotations(pos4);
        // }, () -> {
        // arm.setTurretMotorVoltage(0);
        // arm.setJoint1MotorVoltage(0);
        // arm.setJoint2MotorVoltage(0);
        // arm.setJoint3MotorVoltage(0);
        // arm.setWristMotorVoltage(0);
        // }, arm);

        testCommand = new SequentialCommandGroup(new MoveArmUnsafe(arm, Positions.Pose3d.fromRobotSpace(new Translation3d(0, 1, 0.75)), 100, 0)
        // new WaitCommand(0.5),
        // new MoveArmUnsafe(arm, Positions.Pose3d.fromRobotSpace(new Translation3d(1.3, 0, 0.5)), 0,
        // 0),
        // new WaitCommand(0.5),
        // new MoveArmUnsafe(arm, Positions.Pose3d.fromRobotSpace(new Translation3d(.7, 0, 0.2)),
        // (Math.PI / 2), (Math.PI / 4)),
        // new WaitCommand(0.5)
        );
    }

    private void configureBindings() {
        driverXBox.rightBumper.prioritize(AxisPriorities.DRIVE).getTrigger().onTrue(new InstantCommand(drivetrain::highGear));
        driverXBox.leftBumper.prioritize(AxisPriorities.DRIVE).getTrigger().onTrue(new InstantCommand(drivetrain::lowGear));

        launchpad.missileB.getTrigger().whileTrue(homeArm);

        launchpad.buttonC.getTrigger().toggleOnTrue(turretManual);
        launchpad.buttonC.commandBind(turretManual);

        launchpad.buttonB.getTrigger().toggleOnTrue(joint1Manual);
        launchpad.buttonB.commandBind(joint1Manual);

        launchpad.buttonA.getTrigger().toggleOnTrue(joint2Manual);
        launchpad.buttonA.commandBind(joint2Manual);

        launchpad.buttonF.getTrigger().toggleOnTrue(joint3Manual);
        launchpad.buttonF.commandBind(joint3Manual);

        launchpad.buttonE.getTrigger().toggleOnTrue(wristManual);
        launchpad.buttonE.commandBind(wristManual);

        launchpad.buttonI.getTrigger().toggleOnTrue(fancyArmMo);
        launchpad.buttonI.commandBind(fancyArmMo);

        launchpad.buttonG.getTrigger().whileTrue(testCommand);
        launchpad.buttonG.commandBind(testCommand);

        launchpad.buttonD.getTrigger().onTrue(new InstantCommand(arm::toggleClamp));
        launchpad.buttonD.booleanSupplierBind(arm::getClampSolenoidState);

        launchpad.buttonH.getTrigger().whileTrue(new MoveArmHome(arm));
        launchpad.buttonH.pressBind();
    }

    private void setDefaultCommands() {
        drivetrain.setDefaultCommand(arcadeDrive);
    }

    private void initLogging() {
        // scheduler.schedule(new LogComponents(arm));
        scheduler.schedule(new LogComponents(drivetrain, arm));

    }

    private void initTelemetry() {
        SmartDashboard.putData("Arm", arm);
        SmartDashboard.putData("Drivetrain", drivetrain);
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }

    public void teleopInit() {
        arm.stop();
        System.out.println(MovementMap.generatePathBetweenTwoPoints(Positions.Pose3d.fromRobotSpace(new Translation3d(0.5, 0.5, 0)), Positions.Pose3d.fromRobotSpace(new Translation3d(1, 4.5, 0)),
                        MovementMap.getInstance().getMainMap()));
    }

    public void disabledInit() {
        scheduler.cancelAll();
        arm.stop();
    }

    public void robotPeriodic() {
    }
}
