// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import java.util.Map;
import org.photonvision.PhotonCamera;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.arm.DefaultArmCommand;
import frc.robot.commands.arm.FullManualArm;
import frc.robot.commands.arm.MoveArmUnsafe;
import frc.robot.commands.arm.MovePositionsLaunchpad;
import frc.robot.commands.auto.ArmOutOfStart;
import frc.robot.commands.auto.MoveNBalance;
import frc.robot.commands.auto.Place;
import frc.robot.commands.auto.PlaceNBalance;
import frc.robot.commands.auto.PlaceNMove;
import frc.robot.commands.auto.PlaceNMoveNBalance;
import frc.robot.commands.auto.PlaceNMoveNGrabNPlace;
import frc.robot.commands.auto.PlaceNMoveNGrabNPlace.Type;
import frc.robot.commands.automovements.AutoPickup;
import frc.robot.commands.automovements.NewLimelightPlace;
import frc.robot.commands.automovements.AutoPickup.ELEMENT_TYPE;
import frc.robot.commands.automovements.AutoPickup.LOCATION;
import frc.robot.commands.drivetrain.ArcadeDrive;
import frc.robot.commands.drivetrain.ChargeBalance;
import frc.robot.commands.drivetrain.EncoderDrive;
import frc.robot.commands.drivetrain.ChargeBalance.BalanceDirection;
import frc.robot.devices.Lidar;
import frc.robot.devices.LidarV3Jack;
import frc.robot.devices.PCM;
import frc.robot.devices.LEDs.LEDCalls;
import frc.robot.oi.drivers.ControllerDriver;
import frc.robot.oi.drivers.LaunchpadDriver;
import frc.robot.oi.drivers.ShuffleboardDriver;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIntake;
import frc.robot.subsystems.arm.ArmIntake.State;
import frc.robot.subsystems.arm.ArmPositions.ARM_POSITION;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utilities.lists.AxisPriorities;
import frc.robot.utilities.lists.Ports;
import frc.robot.commands.Home;
import frc.robot.commands.TimedMoveMotor;

public class RobotContainer {

    // Overall
    private CommandScheduler scheduler;
    private AHRS navx;
    private PCM pcm;
    private Lidar gripperLidar;
    private final LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Auto Routine");

    // OI
    private ControllerDriver driverXBox;
    private ControllerDriver gunnerXBox;
    private LaunchpadDriver launchpad;

    // Subsystems
    private Drivetrain drivetrain;
    private Arm arm;
    private ArmIntake armIntake;

    // Commands
    private Command arcadeDrive;

    private Command turretManual;
    private Command joint1Manual;
    private Command joint2Manual;
    private Command joint3Manual;
    private Command intakeManual;

    private Command launchPadArmSelector;

    private Command homeArm;

    // private Command testCommand;

    private boolean altMode = false;

    // private AprilTagCameraWrapper backLeft;
    // private AprilTagCameraWrapper backRight;
    // private AprilTagCameraWrapper front;

    private PhotonCamera gripperCam;

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
        gripperLidar = new LidarV3Jack(Port.kMXP);
        
        // Subsystems
        drivetrain = Drivetrain.init(navx, new Pose2d());
        arm = new Arm(gripperLidar);
        armIntake = new ArmIntake();

        pcm = new PCM(Ports.Other.PCM, drivetrain);

        // backLeft = new AprilTagCameraWrapper("backLeft", new Transform3d(new Translation3d(-0.3175,0.307137,0.231978+0.003175), new Rotation3d(basis, backLeftVec)));   //68.7
        // backRight = new AprilTagCameraWrapper("backRight", new Transform3d(new Translation3d(-0.3175,-0.307137,0.231978+0.003175), new Rotation3d(basis, backRightVec)));  //68.7

        // backLeft = new AprilTagCameraWrapper("backLeft", new Transform3d(new Translation3d(-0.3302,0.307137,0.235153), new Rotation3d(0,Math.toRadians(-25.3), Math.toRadians(90))));  //68.7
        // backRight = new AprilTagCameraWrapper("backRight", new Transform3d(new Translation3d(-0.3302,-0.307137,0.235153), new Rotation3d(0,Math.toRadians(-25.3), Math.toRadians(-90))));  //68.7
        // front = new AprilTagCameraWrapper("front", new Transform3d(new Translation3d(0.3683,0.2159,0.24765), new Rotation3d(0,Math.toRadians(-15), 0)));  //68.7

        gripperCam = new PhotonCamera("Arducam_16MP");
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

        turretManual = new FullManualArm(arm, armIntake, FullManualArm.Type.TURRET, gunnerXBox);
        joint1Manual = new FullManualArm(arm, armIntake, FullManualArm.Type.JOINT_1, gunnerXBox);
        joint2Manual = new FullManualArm(arm, armIntake, FullManualArm.Type.JOINT_2, gunnerXBox);
        joint3Manual = new FullManualArm(arm, armIntake, FullManualArm.Type.JOINT_3, gunnerXBox);
        intakeManual = new FullManualArm(arm, armIntake, FullManualArm.Type.INTAKE, gunnerXBox);

        homeArm = new SequentialCommandGroup(
            new Home(arm.getHomeables()[3]),
            new TimedMoveMotor(arm::setJoint3MotorVoltage, -3, 0.25),
            new Home(arm.getHomeables()[2], arm.getHomeables()[1]),
            new ParallelCommandGroup(new TimedMoveMotor(arm::setJoint1MotorVoltage, 2, 0.2), new TimedMoveMotor(arm::setJoint2MotorVoltage, 2, 0.2)), new Home(gunnerXBox.buttonB.getTrigger()::getAsBoolean, arm.getHomeables()[0]),
            new TimedMoveMotor(arm::setTurretMotorVoltage, 5, 0.1), new MoveArmUnsafe(arm, ARM_POSITION.HOME)
            );

        // homeArm = new SequentialCommandGroup(
        //     new MoveArmUnsafe(arm, ARM_POSITION.PRE_HOME),
        //     new Home(arm)
        // );

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

        driverXBox.buttonX.getTrigger().whileTrue(new NewLimelightPlace(drivetrain, arm));

        driverXBox.buttonA.getTrigger().onTrue(new SequentialCommandGroup(
            new InstantCommand(() -> {
                if (launchpad.funLeft.get()) {
                    AutoPickup.setType(ELEMENT_TYPE.CONE);
                } else if (launchpad.funRight.get()) {
                    AutoPickup.setType(ELEMENT_TYPE.QUORB);
                } else {
                    AutoPickup.setType(ELEMENT_TYPE.NONE);
                }
            }),
            new AutoPickup(arm, armIntake, LOCATION.GROUND).unless(AutoPickup::isNone)
        ).handleInterrupt(LEDCalls.INTAKE_DOWN::cancel).unless(() -> armIntake.getState() != State.STATIONARY));
        driverXBox.buttonA.getTrigger().onTrue(new SequentialCommandGroup(
            new InstantCommand(() -> armIntake.setState(State.STATIONARY), armIntake),
            new MoveArmUnsafe(arm, ARM_POSITION.GROUND_PICKUP_SAFE),
            new MoveArmUnsafe(arm, ARM_POSITION.HOME),
            new InstantCommand(LEDCalls.INTAKE_DOWN::cancel)
        ).handleInterrupt(LEDCalls.INTAKE_DOWN::cancel).unless(() -> armIntake.getState() != State.INTAKE));

        driverXBox.buttonB.getTrigger().onTrue(new SequentialCommandGroup(
            new InstantCommand(() -> {
                if (launchpad.funLeft.get()) {
                    AutoPickup.setType(ELEMENT_TYPE.CONE);
                } else if (launchpad.funRight.get()) {
                    AutoPickup.setType(ELEMENT_TYPE.QUORB);
                } else {
                    AutoPickup.setType(ELEMENT_TYPE.NONE);
                }
            }),
            new AutoPickup(arm, armIntake, LOCATION.LOADING_STATION).unless(AutoPickup::isNone)
        ).handleInterrupt(LEDCalls.INTAKE_DOWN::cancel).unless(() -> armIntake.getState() != State.STATIONARY));
        driverXBox.buttonB.getTrigger().onTrue(new SequentialCommandGroup(
            new InstantCommand(() -> armIntake.setState(State.STATIONARY), armIntake),
            new MoveArmUnsafe(arm, ARM_POSITION.HOME),
            new InstantCommand(LEDCalls.INTAKE_DOWN::cancel)
        ).handleInterrupt(LEDCalls.INTAKE_DOWN::cancel).unless(() -> armIntake.getState() != State.INTAKE));
        driverXBox.buttonB.getTrigger().onTrue(new SequentialCommandGroup(
            new MoveArmUnsafe(arm, ARM_POSITION.HOME),
            new InstantCommand(LEDCalls.INTAKE_DOWN::cancel)
        ).handleInterrupt(LEDCalls.INTAKE_DOWN::cancel).unless(() -> armIntake.getState() != State.STALLING));

        launchpad.missileA.getTrigger().whileTrue(new ChargeBalance(drivetrain, BalanceDirection.FORWARD));
        launchpad.missileB.getTrigger().whileTrue(new StartEndCommand(() -> arm.setAllSoftLimit(false), () -> arm.setAllSoftLimit(true)));
        
        gunnerXBox.buttonY.getTrigger().whileTrue(new MoveArmUnsafe(arm, ARM_POSITION.HOME));
        gunnerXBox.buttonA.getTrigger().onTrue(new SelectCommand(Map.ofEntries(
            Map.entry(State.OTHER, new InstantCommand(() -> armIntake.setState(State.STATIONARY), armIntake)),
            Map.entry(State.INTAKE, new InstantCommand(() -> armIntake.setState(State.STATIONARY), armIntake)),
            Map.entry(State.OUTTAKE, new InstantCommand(() -> armIntake.setState(State.STATIONARY), armIntake)),
            Map.entry(State.STATIONARY, new InstantCommand(() -> armIntake.setState(State.INTAKE), armIntake)),
            Map.entry(State.STALLING, new SequentialCommandGroup(
                new InstantCommand(() -> armIntake.setState(State.OUTTAKE), armIntake),
                new WaitCommand(0.25),
                new InstantCommand(() -> armIntake.setState(State.STATIONARY), armIntake)
            ))
        ), 
        armIntake::getState));
        gunnerXBox.buttonX.getTrigger().whileTrue(new MoveArmUnsafe(arm, ARM_POSITION.HIGH_ASPECT));

        launchpad.buttonA.getTrigger().and(this::notAltMode).toggleOnTrue(joint2Manual);
        launchpad.buttonB.getTrigger().and(this::notAltMode).toggleOnTrue(joint1Manual);
        launchpad.buttonC.getTrigger().and(this::notAltMode).toggleOnTrue(turretManual);
        launchpad.buttonD.getTrigger().onTrue(new SelectCommand(Map.ofEntries(
            Map.entry(State.OTHER, new InstantCommand(() -> armIntake.setState(State.STATIONARY), armIntake)),
            Map.entry(State.INTAKE, new InstantCommand(() -> armIntake.setState(State.STATIONARY), armIntake)),
            Map.entry(State.OUTTAKE, new InstantCommand(() -> armIntake.setState(State.STATIONARY), armIntake)),
            Map.entry(State.STATIONARY, new InstantCommand(() -> armIntake.setState(State.INTAKE), armIntake)),
            Map.entry(State.STALLING, new SequentialCommandGroup(
                new InstantCommand(() -> armIntake.setState(State.OUTTAKE), armIntake),
                new WaitCommand(0.25),
                new InstantCommand(() -> armIntake.setState(State.STATIONARY), armIntake)
            ))
        ), 
        armIntake::getState));
        launchpad.buttonE.getTrigger().and(this::notAltMode).toggleOnTrue(intakeManual);
        launchpad.buttonF.getTrigger().and(this::notAltMode).toggleOnTrue(joint3Manual);
        launchpad.buttonG.getTrigger().toggleOnTrue(launchPadArmSelector);
        launchpad.buttonH.getTrigger().and(this::notAltMode).and(() -> !launchpad.missileB.getTrigger().getAsBoolean()).whileTrue(homeArm);
        launchpad.buttonH.getTrigger().and(this::notAltMode).and(() -> launchpad.missileB.getTrigger().getAsBoolean()).whileTrue(new SequentialCommandGroup(
            new MoveArmUnsafe(arm, ARM_POSITION.PRE_HOME),
            new Home(arm),
            new MoveArmUnsafe(arm, ARM_POSITION.HOME)
        ));
        launchpad.buttonI.getTrigger().and(this::notAltMode).onTrue(new InstantCommand(AutoPickup::toggleType));
        
        launchpad.buttonA.commandBind(joint2Manual);
        launchpad.buttonB.commandBind(joint1Manual);
        launchpad.buttonC.commandBind(turretManual);
        launchpad.buttonD.booleanSupplierBind(() -> !(armIntake.getState() == State.STATIONARY));
        launchpad.buttonE.commandBind(intakeManual);
        launchpad.buttonF.commandBind(joint3Manual);
        launchpad.buttonG.commandBind(launchPadArmSelector);
        launchpad.buttonH.commandBind(homeArm);
        launchpad.buttonI.booleanSupplierBind(AutoPickup::isQuorb);
    }

    private void setDefaultCommands() {
        drivetrain.setDefaultCommand(arcadeDrive);
        arm.setDefaultCommand(new DefaultArmCommand(arm));
    }

    private void initLogging() {
        Logger.getInstance().recordMetadata("ProjectName", "MyProject"); // Set a metadata value

        if (RobotBase.isReal()) {
            Logger.getInstance().addDataReceiver(new WPILOGWriter("/media/sda1/")); // Log to a USB stick
            Logger.getInstance().addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
            // new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
        } else {
            // setUseTiming(false); // Run as fast as possible
            String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
            Logger.getInstance().setReplaySource(new WPILOGReader(logPath)); // Read replay log
            Logger.getInstance().addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
        }
        
        // Logger.getInstance().disableDeterministicTimestamps() // See "Deterministic Timestamps" in the "Understanding Data Flow" page
        Logger.getInstance().start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.
    }

    private void initTelemetry() {
        SmartDashboard.putData("Arm", arm);
        SmartDashboard.putData("Drivetrain", drivetrain);
        SmartDashboard.putData("PCM", pcm);
        SmartDashboard.putData("Lidar", gripperLidar);
        SmartDashboard.putData("ArmIntake", armIntake);
    }

    public void createAutoCommands() {
        autoChooser.addDefaultOption("DoNothing", new ArmOutOfStart(arm));
        autoChooser.addOption("Move", new ParallelCommandGroup(
            new SequentialCommandGroup(
                new ArmOutOfStart(arm),
                new MoveArmUnsafe(arm, ARM_POSITION.HOME)
            ),
            new EncoderDrive(1.5, drivetrain)
        ));
        autoChooser.addOption("Place", new Place(arm, drivetrain));
        autoChooser.addOption("PlaceNMove", new PlaceNMove(drivetrain, arm));
        autoChooser.addOption("Balance", new SequentialCommandGroup(
            new ArmOutOfStart(arm),
            new ChargeBalance(drivetrain, BalanceDirection.FORWARD)
        ));
        autoChooser.addOption("BackwardsBalance", new SequentialCommandGroup(
            new ArmOutOfStart(arm),
            new ChargeBalance(drivetrain, BalanceDirection.BACKWARD)
        ));
        autoChooser.addOption("PlaceNBalance", new PlaceNBalance(drivetrain, arm));
        autoChooser.addOption("MoveNBalance", new MoveNBalance(arm, drivetrain));
        autoChooser.addOption("PlaceNMoveNBalance", new PlaceNMoveNBalance(arm, drivetrain));
        // ShuffleboardDriver.autoChooser.addOption("PlaceNMoveNGrab", new PlaceNMoveNGrab(arm, drivetrain, quorbCamera, coneCamera));
        autoChooser.addOption("CloseBluePlaceNMoveNGrabNPlace", new PlaceNMoveNGrabNPlace(arm, drivetrain, Type.CloseToSubstation, Alliance.Blue));
        autoChooser.addOption("FarBluePlaceNMoveNGrabNPlace", new PlaceNMoveNGrabNPlace(arm, drivetrain, Type.FarFromSubstation, Alliance.Blue));
        autoChooser.addOption("CloseRedPlaceNMoveNGrabNPlace", new PlaceNMoveNGrabNPlace(arm, drivetrain, Type.CloseToSubstation, Alliance.Red));
        autoChooser.addOption("FarRedPlaceNMoveNGrabNPlace", new PlaceNMoveNGrabNPlace(arm, drivetrain, Type.FarFromSubstation, Alliance.Red));
    }

    public Command getAutonomousCommand() {
        return autoChooser.get();
    }

    public void robotInit() {
        initLogging();
        pcm.enableCompressorAnalog(80, 100);
        ShuffleboardDriver.init();
        LEDCalls.ON.activate();

        // TODO - check port number and comment out server for comp
        // PathPlannerServer.startServer(5468);
    }
    public void robotPeriodic() {
        // String val = HPSelector.get();

        if (launchpad.funRight.get()) {
            LEDCalls.CUBE_HP.activate();
            LEDCalls.CONE_HP.cancel();
        } else if (launchpad.funLeft.get()) {
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
        // (new MoveArmUnsafe(arm, ARM_POSITION.STARTING_CONFIG, true)).initialize();
        // (new SequentialCommandGroup(
        //     new InstantCommand(arm::unclamp),
        //     new WaitCommand(0.25),
        //     new WaitCommand(0.25),
        //     new InstantCommand(arm::clamp)
        // )).initialize();
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
