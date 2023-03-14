// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import org.photonvision.PhotonCamera;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.arm.ArmMO;
import frc.robot.commands.arm.DefaultArmCommand;
import frc.robot.commands.arm.FullManualArm;
import frc.robot.commands.arm.MoveArmToNode;
import frc.robot.commands.arm.MoveArmUnsafe;
import frc.robot.commands.arm.MovePositionsLaunchpad;
import frc.robot.commands.arm.MoveToPickupSubstation;
import frc.robot.commands.auto.ArmOutOfStart;
import frc.robot.commands.auto.MoveNBalance;
import frc.robot.commands.auto.Place;
import frc.robot.commands.auto.PlaceNBalance;
import frc.robot.commands.auto.PlaceNMove;
import frc.robot.commands.auto.PlaceNMoveNBalance;
import frc.robot.commands.auto.PlaceNMoveNGrab;
import frc.robot.commands.auto.PlaceNMoveNGrabNPlace;
import frc.robot.commands.auto.PlaceNMoveNGrabNPlace.Type;
import frc.robot.commands.drivetrain.ArcadeDrive;
import frc.robot.commands.drivetrain.BackwardsBalance;
import frc.robot.commands.drivetrain.ChargeStationBalance;
import frc.robot.commands.drivetrain.EncoderDrive;
import frc.robot.devices.Lidar;
import frc.robot.devices.LidarV3;
import frc.robot.devices.PCM;
import frc.robot.devices.LEDs.LEDCalls;
import frc.robot.oi.drivers.ControllerDriver;
import frc.robot.oi.drivers.LaunchpadDriver;
import frc.robot.oi.drivers.ShuffleboardDriver;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmPositions.ARM_POSITION;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utilities.lists.AxisPriorities;
import frc.robot.utilities.lists.Ports;
import frc.robot.commands.Home;
import frc.robot.commands.LogComponents;
import frc.robot.commands.TimedMoveMotor;

public class RobotContainer {

    // Overall
    private CommandScheduler scheduler;
    private AHRS navx;
    private PCM pcm;
    private Lidar gripperLidar;

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
    private Command fancyArmMo;

    private Command launchPadArmSelector;

    private Command homeArm;

    // private Command testCommand;

    private boolean altMode = false;

    // private AprilTagCameraWrapper backLeft;
    // private AprilTagCameraWrapper backRight;
    // private AprilTagCameraWrapper front;

    private PhotonCamera quorbCamera;
    private PhotonCamera coneCamera;

    Trajectory blueHigh;
    Trajectory redHigh;
    Trajectory blueMid;
    Trajectory redMid;
    Trajectory blueLow;
    Trajectory redLow;


    StringSubscriber HPSelector;

    public RobotContainer() throws IOException {
        scheduler = CommandScheduler.getInstance();

        HPSelector = NetworkTableInstance.getDefault().getTable("customDS").getStringTopic("indicator").subscribe("");

        // OI
        driverXBox = new ControllerDriver(Ports.OI.DRIVER_XBOX_PORT);
        gunnerXBox = new ControllerDriver(Ports.OI.GUNNER_XBOX_PORT);
        launchpad = new LaunchpadDriver(Ports.OI.LAUNCHPAD_PORT);

        // Devices
        navx = new AHRS();
        gripperLidar = new LidarV3();
        
        // Subsystems
        drivetrain = Drivetrain.init(navx, new Pose2d());
        arm = new Arm(gripperLidar);

        pcm = new PCM(Ports.Other.PCM, drivetrain);

        // backLeft = new AprilTagCameraWrapper("backLeft", new Transform3d(new Translation3d(-0.3175,0.307137,0.231978+0.003175), new Rotation3d(basis, backLeftVec)));   //68.7
        // backRight = new AprilTagCameraWrapper("backRight", new Transform3d(new Translation3d(-0.3175,-0.307137,0.231978+0.003175), new Rotation3d(basis, backRightVec)));  //68.7

        // backLeft = new AprilTagCameraWrapper("backLeft", new Transform3d(new Translation3d(-0.3302,0.307137,0.235153), new Rotation3d(0,Math.toRadians(-25.3), Math.toRadians(90))));  //68.7
        // backRight = new AprilTagCameraWrapper("backRight", new Transform3d(new Translation3d(-0.3302,-0.307137,0.235153), new Rotation3d(0,Math.toRadians(-25.3), Math.toRadians(-90))));  //68.7
        // front = new AprilTagCameraWrapper("front", new Transform3d(new Translation3d(0.3683,0.2159,0.24765), new Rotation3d(0,Math.toRadians(-15), 0)));  //68.7

        quorbCamera = new PhotonCamera("QuorbGrabber");
        coneCamera = new PhotonCamera("CubeGrabber");

        createCommands();
        createAutoCommands();
        setDefaultCommands();
        configureBindings();

        // Init Logging and Telemetry
        initLogging();
        initTelemetry();
    }

    private void createCommands() {
        arcadeDrive = new ArcadeDrive(drivetrain, driverXBox.rightTrigger, driverXBox.leftTrigger, driverXBox.leftX, driverXBox.buttonY);

        turretManual = new FullManualArm(arm, FullManualArm.Type.TURRET, gunnerXBox);
        joint1Manual = new FullManualArm(arm, FullManualArm.Type.JOINT_1, gunnerXBox);
        joint2Manual = new FullManualArm(arm, FullManualArm.Type.JOINT_2, gunnerXBox);
        joint3Manual = new FullManualArm(arm, FullManualArm.Type.JOINT_3, gunnerXBox);
        fancyArmMo = new ArmMO(arm, gunnerXBox, launchpad);

        homeArm = new SequentialCommandGroup(
            new Home(arm.getHomeables()[3]),
            new TimedMoveMotor(arm::setJoint3MotorVoltage, -3, 0.25),
            new Home(arm.getHomeables()[2], arm.getHomeables()[1]),
            new ParallelCommandGroup(new TimedMoveMotor(arm::setJoint1MotorVoltage, 2, 0.2), new TimedMoveMotor(arm::setJoint2MotorVoltage, 2, 0.2)), new Home(gunnerXBox.buttonB.getTrigger()::getAsBoolean, arm.getHomeables()[0]),
            new TimedMoveMotor(arm::setTurretMotorVoltage, 5, 0.1), new MoveArmUnsafe(arm, ARM_POSITION.HOME)
            );

        launchPadArmSelector = new MovePositionsLaunchpad(arm, launchpad, this);

        // homeArm = new SequentialCommandGroup(new Home(arm.getHomeables()[4]), new Home(arm.getHomeables()[3]),
        //     new ParallelCommandGroup(new TimedMoveMotor(arm::setWristMotorVoltage, -12, 0.25), new TimedMoveMotor(arm::setJoint3MotorVoltage, -3, 0.25)),
        //     new Home(arm.getHomeables()[2], arm.getHomeables()[1]));
        // testCommand = new SequentialCommandGroup(new MoveArmUnsafe(arm, Positions.Pose3d.fromOtherSpace(new Translation3d(-0.1, -0.5, 0.75), Arm.ROBOT_TO_TURRET_BASE), 0, (Math.PI / 4)));
        // testCommand = new SubstationPickup(drivetrain, arm, Side.Left);
        // testCommand = new FollowDynamicTrajectory(drivetrain::getPose, () -> new Pose2d(new Translation2d(1, 1), new Rotation2d()), () -> new ArrayList<>(), drivetrain.generateTrajectoryConfigHighGear(), drivetrain);
    }

    private void configureBindings() {
        driverXBox.rightBumper.prioritize(AxisPriorities.DRIVE).getTrigger().onTrue(new InstantCommand(drivetrain::highGear));
        driverXBox.leftBumper.prioritize(AxisPriorities.DRIVE).getTrigger().onTrue(new InstantCommand(drivetrain::lowGear));

        driverXBox.dPadLeft.getTrigger().onTrue(new SequentialCommandGroup(
            new InstantCommand(arm::unclamp),
            new MoveToPickupSubstation(arm, frc.robot.commands.arm.MoveToPickupSubstation.Side.LEFT),
            new InstantCommand(arm::clamp),
            new WaitCommand(0.5),
            new MoveArmUnsafe(arm, ARM_POSITION.HIGH_ASPECT)
        ));

        driverXBox.dPadRight.getTrigger().onTrue(new SequentialCommandGroup(
            new InstantCommand(arm::unclamp),
            new MoveToPickupSubstation(arm, frc.robot.commands.arm.MoveToPickupSubstation.Side.RIGHT),
            new InstantCommand(arm::clamp),
            new WaitCommand(0.5),
            new MoveArmUnsafe(arm, ARM_POSITION.HIGH_ASPECT)
        ));

        // driverXBox.buttonB.getTrigger().whileTrue(new MoveArmUnsafe(arm, ARM_POSITION.HOME));

        driverXBox.buttonX.getTrigger().whileTrue(new MoveArmToNode(arm));

        launchpad.missileA.getTrigger().whileTrue(new BackwardsBalance(drivetrain));
        // launchpad.missileA.getTrigger().whileTrue(new AutoPickup(drivetrain, arm, QuorbCamera, QuorbCamera));
        // launchpad.missileA.getTrigger().whileTrue(new MeasureSpinSpin(drivetrain, 0.05, 0.75, 0.025));
        launchpad.missileB.getTrigger().whileTrue(new StartEndCommand(() -> arm.setAllSoftLimit(false), () -> arm.setAllSoftLimit(true)));
        
        gunnerXBox.buttonY.getTrigger().whileTrue(new MoveArmUnsafe(arm, ARM_POSITION.HOME));
        gunnerXBox.buttonA.getTrigger().onTrue(new InstantCommand(arm::toggleClamp));
        gunnerXBox.buttonX.getTrigger().whileTrue(new MoveArmUnsafe(arm, ARM_POSITION.HIGH_ASPECT));

        launchpad.buttonA.getTrigger().and(this::notAltMode).toggleOnTrue(joint2Manual);
        launchpad.buttonB.getTrigger().and(this::notAltMode).toggleOnTrue(joint1Manual);
        launchpad.buttonC.getTrigger().and(this::notAltMode).toggleOnTrue(turretManual);
        launchpad.buttonD.getTrigger().onTrue(new InstantCommand(arm::toggleClamp));
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
        launchpad.buttonI.commandBind(fancyArmMo);
        launchpad.buttonD.booleanSupplierBind(arm::getClampSolenoidState);
    }

    private void setDefaultCommands() {
        drivetrain.setDefaultCommand(arcadeDrive);
        arm.setDefaultCommand(new DefaultArmCommand(arm));
    }

    private void initLogging() {
        scheduler.schedule(new LogComponents(drivetrain, arm));
        DriverStation.startDataLog(DataLogManager.getLog());
    }

    private void initTelemetry() {
        SmartDashboard.putData("Arm", arm);
        SmartDashboard.putData("Drivetrain", drivetrain);
        SmartDashboard.putData("PCM", pcm);
        SmartDashboard.putData("Lidar", gripperLidar);
    }

    public void createAutoCommands() {
        ShuffleboardDriver.autoChooser.setDefaultOption("Move", new ParallelCommandGroup(
            new SequentialCommandGroup(
                new ArmOutOfStart(arm),
                new MoveArmUnsafe(arm, ARM_POSITION.HOME)
            ),
            new EncoderDrive(1.5, drivetrain)
        ));
        ShuffleboardDriver.autoChooser.addOption("DoNothing", new ArmOutOfStart(arm));
        ShuffleboardDriver.autoChooser.addOption("Place", new Place(arm, drivetrain));
        ShuffleboardDriver.autoChooser.addOption("PlaceNMove", new PlaceNMove(drivetrain, arm));
        ShuffleboardDriver.autoChooser.addOption("Balance", new SequentialCommandGroup(
            new ArmOutOfStart(arm),
            new ChargeStationBalance(drivetrain)
        ));
        ShuffleboardDriver.autoChooser.addOption("BackwardsBalance", new SequentialCommandGroup(
            new ArmOutOfStart(arm),
            new BackwardsBalance(drivetrain)
        ));
        ShuffleboardDriver.autoChooser.addOption("PlaceNBalance", new PlaceNBalance(drivetrain, arm));
        ShuffleboardDriver.autoChooser.addOption("MoveNBalance", new MoveNBalance(arm, drivetrain));
        ShuffleboardDriver.autoChooser.addOption("PlaceNMoveNBalance", new PlaceNMoveNBalance(arm, drivetrain));
        ShuffleboardDriver.autoChooser.addOption("PlaceNMoveNGrab", new PlaceNMoveNGrab(arm, drivetrain, quorbCamera, coneCamera));
        ShuffleboardDriver.autoChooser.addOption("Close PlaceNMoveNGrabNPlace", new PlaceNMoveNGrabNPlace(arm, drivetrain, Type.CloseToSubstation));
        ShuffleboardDriver.autoChooser.addOption("Far PlaceNMoveNGrabNPlace", new PlaceNMoveNGrabNPlace(arm, drivetrain, Type.FarFromSubstation));

    }

    public Command getAutonomousCommand() {
        return ShuffleboardDriver.autoChooser.getSelected();
    }

    public void robotInit() {
        pcm.enableCompressorAnalog(80, 120);
        ShuffleboardDriver.init();
        LEDCalls.ON.activate();
        try {
            blueHigh = TrajectoryUtil.fromPathweaverJson(Filesystem.getDeployDirectory().toPath().resolve("paths/BlueBottom.wpilib.json"));
            blueMid = TrajectoryUtil.fromPathweaverJson(Filesystem.getDeployDirectory().toPath().resolve("paths/BlueMid.wiplib.json"));
            blueLow = TrajectoryUtil.fromPathweaverJson(Filesystem.getDeployDirectory().toPath().resolve("paths/BlueTop.wpilib.json"));
            redHigh = TrajectoryUtil.fromPathweaverJson(Filesystem.getDeployDirectory().toPath().resolve("paths/RedBottom.wpilib.json"));
            redMid = TrajectoryUtil.fromPathweaverJson(Filesystem.getDeployDirectory().toPath().resolve("paths/RedMid.wpilib.json"));
            redLow = TrajectoryUtil.fromPathweaverJson(Filesystem.getDeployDirectory().toPath().resolve("paths/RedTop.wpilib.json"));
        } catch (IOException e) {
            e.printStackTrace();
        }
        // TODO - check port number and comment out server for comp
        PathPlannerServer.startServer(5468);
    }
    public void robotPeriodic() {
        String val = HPSelector.get();

        if (val.equalsIgnoreCase("cube")) {
            LEDCalls.CUBE_HP.activate();
            LEDCalls.CONE_HP.cancel();
        } else if (val.equalsIgnoreCase("cone")) {
            LEDCalls.CUBE_HP.cancel();
            LEDCalls.CONE_HP.activate();
        } else {
            LEDCalls.CUBE_HP.cancel();
            LEDCalls.CONE_HP.cancel();
        }
        // System.out.println(gripperLidar.getAverageDistance());
    }

    public void disabledInit() {
        scheduler.cancelAll();
        arm.stop();
        drivetrain.stop();
    }
    public void disabledPeriodic() {}
    public void disabledExit() {}

    public void autonomousInit() {
    }
    public void autonomousPeriodic() {}
    public void autonomousExit() {}

    public void teleopInit() {
        // front.forceDisableDriverMode();
        // backRight.forceDisableDriverMode();
        // backLeft.forceDisableDriverMode();
        // drivetrain.addVisionCamera(front);
        // drivetrain.addVisionCamera(backRight);
        // drivetrain.addVisionCamera(backLeft);
    }
    public void teleopPeriodic() {
    }
    public void teleopExit() {}

    public void testInit() {
       (new MoveArmUnsafe(arm, ARM_POSITION.STARTING_CONFIG, true)).initialize();
    }
    public void testPeriodic() {}
    public void testExit() {
        arm.stop();
    }

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
