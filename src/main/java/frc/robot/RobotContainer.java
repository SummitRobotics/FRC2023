// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.util.ArrayList;
import java.util.function.BooleanSupplier;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.arm.ArmMO;
import frc.robot.commands.arm.FullManualArm;
import frc.robot.commands.arm.MoveArmHighAspect;
import frc.robot.commands.arm.MoveArmHome;
import frc.robot.commands.arm.MoveArmUnsafe;
import frc.robot.commands.arm.MovePositionsLaunchpad;
import frc.robot.commands.automovements.SubstationPickup;
import frc.robot.commands.automovements.SubstationPickup.Side;
import frc.robot.commands.drivetrain.ArcadeDrive;
import frc.robot.commands.drivetrain.ChargeStationBalance;
import frc.robot.commands.drivetrain.FollowDynamicTrajectory;
import frc.robot.commands.drivetrain.FollowDynamicTrajectoryThreaded;
import frc.robot.devices.AprilTagCameraWrapper;
import frc.robot.devices.PCM;
import frc.robot.devices.LEDs.LEDCall;
import frc.robot.devices.LEDs.LEDRange;
import frc.robot.devices.LEDs.LEDs;
import frc.robot.oi.drivers.ControllerDriver;
import frc.robot.oi.drivers.LaunchpadDriver;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utilities.Positions;
import frc.robot.utilities.lists.AxisPriorities;
import frc.robot.utilities.lists.Colors;
import frc.robot.utilities.lists.FieldElementPositions;
import frc.robot.utilities.lists.Ports;
import frc.robot.commands.Home;
import frc.robot.commands.LogComponents;
import frc.robot.commands.TimedMoveMotor;

public class RobotContainer {

    // Overall
    private CommandScheduler scheduler;
    private AHRS navx;
    private PCM pcm;

    // OI
    private ControllerDriver driverXBox;
    private ControllerDriver gunnerXBox;
    private LaunchpadDriver launchpad;

    // Subsystems
    private Drivetrain drivetrain;
    private Arm arm;

    // Commands
    private Command arcadeDrive;
    private Command balance;

    private Command turretManual;
    private Command joint1Manual;
    private Command joint2Manual;
    private Command joint3Manual;
    private Command wristManual;
    private Command fancyArmMo;

    private Command launchPadArmSelector;

    private Command homeArm;

    private Command testCommand;

    private boolean altMode = false;

    // private AprilTagCameraWrapper backLeft;
    // private AprilTagCameraWrapper backRight;

    public RobotContainer() throws IOException {
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

        pcm = new PCM(Ports.Other.PCM, drivetrain);

        // backLeft = new AprilTagCameraWrapper("backLeft", new Transform3d(new Translation3d(-0.324868,0.307137,0.231978), (new Rotation3d(0,0,Math.toRadians(22.8))).plus(new Rotation3d(0,Math.toRadians(67.4),0)).plus(new Rotation3d(Math.toRadians(-8.5),0,0))));
        // backRight = new AprilTagCameraWrapper("backRight", new Transform3d(new Translation3d(-0.324868,-0.307137,0.231978), (new Rotation3d(0,0,Math.toRadians(-22.8))).plus(new Rotation3d(0,Math.toRadians(67.4),0)).plus(new Rotation3d(Math.toRadians(8.5),0,0))));

        // drivetrain.addVisionCamera(backLeft);
        // drivetrain.addVisionCamera(backRight);

        createCommands();
        setDefaultCommands();
        configureBindings();

        // Init Logging and Telemetry
        initLogging();
        initTelemetry();
    }

    private void createCommands() {
        arcadeDrive = new ArcadeDrive(drivetrain, driverXBox.rightTrigger, driverXBox.leftTrigger, driverXBox.leftX, driverXBox.buttonY);
        balance = new ChargeStationBalance(drivetrain, navx);

        turretManual = new FullManualArm(arm, FullManualArm.Type.TURRET, gunnerXBox);
        joint1Manual = new FullManualArm(arm, FullManualArm.Type.JOINT_1, gunnerXBox);
        joint2Manual = new FullManualArm(arm, FullManualArm.Type.JOINT_2, gunnerXBox);
        joint3Manual = new FullManualArm(arm, FullManualArm.Type.JOINT_3, gunnerXBox);
        wristManual = new FullManualArm(arm, FullManualArm.Type.WRIST, gunnerXBox);
        fancyArmMo = new ArmMO(arm, gunnerXBox, launchpad);

        homeArm = new SequentialCommandGroup(new Home(gunnerXBox.buttonB.getTrigger()::getAsBoolean, arm.getHomeables()[4]), new Home(arm.getHomeables()[3]),
            new ParallelCommandGroup(new TimedMoveMotor(arm::setWristMotorVoltage, -12, 0.25), new TimedMoveMotor(arm::setJoint3MotorVoltage, -3, 0.25)),
            new Home(arm.getHomeables()[2], arm.getHomeables()[1]),
            new ParallelCommandGroup(new TimedMoveMotor(arm::setJoint1MotorVoltage, 2, 0.2), new TimedMoveMotor(arm::setJoint2MotorVoltage, 2, 0.2)), new Home(gunnerXBox.buttonB.getTrigger()::getAsBoolean, arm.getHomeables()[0]),
            new TimedMoveMotor(arm::setTurretMotorVoltage, 5, 0.1), new MoveArmHome(arm));

        launchPadArmSelector = new MovePositionsLaunchpad(arm, launchpad, this);

        // homeArm = new SequentialCommandGroup(new Home(arm.getHomeables()[4]), new Home(arm.getHomeables()[3]),
        //     new ParallelCommandGroup(new TimedMoveMotor(arm::setWristMotorVoltage, -12, 0.25), new TimedMoveMotor(arm::setJoint3MotorVoltage, -3, 0.25)),
        //     new Home(arm.getHomeables()[2], arm.getHomeables()[1]));

        // testCommand = new SequentialCommandGroup(new MoveArmUnsafe(arm, Positions.Pose3d.fromOtherSpace(new Translation3d(-0.1, -0.5, 0.75), Arm.ROBOT_TO_TURRET_BASE), 0, (Math.PI / 4)));
        testCommand = new SubstationPickup(drivetrain, arm, Side.Left);
        // testCommand = new FollowDynamicTrajectory(drivetrain::getPose, () -> new Pose2d(new Translation2d(1, 1), new Rotation2d()), () -> new ArrayList<>(), drivetrain.generateTrajectoryConfigHighGear(), drivetrain);
    }

    private void configureBindings() {
        driverXBox.rightBumper.prioritize(AxisPriorities.DRIVE).getTrigger().onTrue(new InstantCommand(drivetrain::highGear));
        driverXBox.leftBumper.prioritize(AxisPriorities.DRIVE).getTrigger().onTrue(new InstantCommand(drivetrain::lowGear));

        // driverXBox.buttonA.getTrigger().onTrue(new MoveArmUnsafe(arm,Positions.Pose3d.fromFieldSpace(FieldElementPositions.RED_LEFT_SUBSTATION),0.0,0.0));

        driverXBox.dPadLeft.getTrigger().onTrue(new SequentialCommandGroup(
            new InstantCommand(arm::unclamp),
            new MoveArmUnsafe(arm,Positions.Pose3d.fromFieldSpace(FieldElementPositions.RED_LEFT_SUBSTATION),0.0,0.0),
            new InstantCommand(arm::clamp),
            new WaitCommand(0.5),
            new MoveArmHighAspect(arm)
        ));

        launchpad.missileA.getTrigger().whileTrue(testCommand);
        launchpad.missileB.getTrigger().whileTrue(new StartEndCommand(() -> arm.setAllSoftLimit(false), () -> arm.setAllSoftLimit(true)));
        
        gunnerXBox.buttonY.getTrigger().whileTrue(new MoveArmHome(arm));
        gunnerXBox.buttonA.getTrigger().onTrue(new InstantCommand(arm::toggleClamp));
        gunnerXBox.buttonX.getTrigger().whileTrue(new MoveArmHighAspect(arm));

        launchpad.buttonA.getTrigger().and(this::notAltMode).toggleOnTrue(joint2Manual);
        launchpad.buttonB.getTrigger().and(this::notAltMode).toggleOnTrue(joint1Manual);
        launchpad.buttonC.getTrigger().and(this::notAltMode).toggleOnTrue(turretManual);
        launchpad.buttonD.getTrigger().onTrue(new InstantCommand(arm::toggleClamp));
        launchpad.buttonE.getTrigger().and(this::notAltMode).toggleOnTrue(wristManual);
        launchpad.buttonF.getTrigger().and(this::notAltMode).toggleOnTrue(joint3Manual);
        launchpad.buttonH.getTrigger().and(this::notAltMode).whileTrue(homeArm);
        launchpad.buttonI.getTrigger().and(this::notAltMode).toggleOnTrue(fancyArmMo);

        launchpad.buttonG.getTrigger().toggleOnTrue(launchPadArmSelector);
        launchpad.buttonG.commandBind(launchPadArmSelector);

        launchpad.buttonH.commandBind(homeArm);
        launchpad.buttonB.commandBind(joint1Manual);
        launchpad.buttonC.commandBind(turretManual);
        launchpad.buttonA.commandBind(joint2Manual);
        launchpad.buttonF.commandBind(joint3Manual);
        launchpad.buttonE.commandBind(wristManual);
        launchpad.buttonI.commandBind(fancyArmMo);
        launchpad.buttonD.booleanSupplierBind(arm::getClampSolenoidState);
    }

    private void setDefaultCommands() {
        drivetrain.setDefaultCommand(arcadeDrive);
    }

    private void initLogging() {
        scheduler.schedule(new LogComponents(drivetrain, arm));
    }

    private void initTelemetry() {
        SmartDashboard.putData("Arm", arm);
        SmartDashboard.putData("Drivetrain", drivetrain);
        SmartDashboard.putData("PCM", pcm);
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }

    public void robotInit() {
        pcm.enableCompressorAnalog(80, 120);
        LEDs.getInstance().addCall("Robot On", new LEDCall(0, LEDRange.All).solid(Colors.GREEN));
    }
    public void robotPeriodic() {}

    public void disabledInit() {
        scheduler.cancelAll();
        arm.stop();
        drivetrain.stop();
    }
    public void disabledPeriodic() {}
    public void disabledExit() {}

    public void autonomousInit() {}
    public void autonomousPeriodic() {}
    public void autonomousExit() {}

    public void teleopInit() {
        drivetrain.setPose(new Pose2d(new Translation2d(3.39, 7.878), new Rotation2d(-2.852, 0)));
        // drivetrain.setPose(new Pose2d(new Translation2d(0, 0), new Rotation2d()));
    }
    public void teleopPeriodic() {}
    public void teleopExit() {}

    public void testInit() {}
    public void testPeriodic() {}
    public void testExit() {}

    private boolean altMode() {
        return altMode;
    }

    private boolean notAltMode() {
        return !altMode();
    }

    public void disableAltMode() {
        altMode = false;
    }

    public void enableAltMode() {
        altMode = true;
    }
}
