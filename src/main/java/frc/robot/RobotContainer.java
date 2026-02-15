//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.robot;

import java.util.function.Supplier;

// CTRE Imports
import com.ctre.phoenix6.swerve.*;

// WPI libraries
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import static edu.wpi.first.units.Units.*;

// Commands
import frc.robot.commands.agitate.*;
import frc.robot.commands.climb.*;
import frc.robot.commands.drive.*;
import frc.robot.commands.feed.*;
import frc.robot.commands.hopper.*;
import frc.robot.commands.intake.*;
import frc.robot.commands.shoot.*;
// Subsystems
import frc.robot.subsystems.*;
import frc.robot.tools.shooterModel.data.*;
import frc.robot.tools.shooterModel.model.*;
// Robot Utilities
import frc.robot.utilities.*;
import frc.robot.generated.*;
import frc.robot.localization.PhotonVision.PVManager;
import frc.robot.localization.nodes.Nodes.Hub;
import frc.robot.localization.nodes.Nodes.Tower;
import frc.robot.localization.quest.QuestNavSubsystem;
// Constants
import frc.robot.Constants.*;

// Autos
import frc.robot.auto.routines.*;

public class RobotContainer {

    // ----------------------------------------------------------
    // Autonomous chooser
    // ----------------------------------------------------------
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    // ----------------------------------------------------------
    // Controllers
    // ----------------------------------------------------------
    private final CommandXboxController bottomDriver = new CommandXboxController(0);
    private final CommandXboxController topDriver = new CommandXboxController(1);

    // ----------------------------------------------------------
    // Telemetry Looger for all data being logged
    // ----------------------------------------------------------
    private final Telemetry logger = new Telemetry();

    // ----------------------------------------------------------
    // Drive configuration
    // ----------------------------------------------------------
    public static double MaxSpeed =
        0.5 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

    public static double MaxAngularRate =
        RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    public final CommandSwerveDrivetrain drivetrain =
        TunerConstants.createDrivetrain();

    private final SwerveRequest.FieldCentric driveRequest =
        new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1)
            .withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);

    private final Command driveCommand =
        drivetrain.applyRequest(() -> driveRequest
            .withVelocityX(-bottomDriver.getLeftY() * MaxSpeed)
            .withVelocityY(-bottomDriver.getLeftX() * MaxSpeed)
            .withRotationalRate(-bottomDriver.getRightX() * MaxAngularRate));

    // ----------------------------------------------------------
    // Subsystems
    // ----------------------------------------------------------

    // -----------------------------
    // Vision
    // -----------------------------
    private final QuestNavSubsystem questNav = new QuestNavSubsystem(drivetrain, logger);

    public final PVManager PVManager =
        new PVManager(
            logger,
            drivetrain::addVisionMeasurement,
            drivetrain::getHeading,
            drivetrain::seedPoseFromVision,
            questNav::setQuestNavPose,
            new PVManager.CameraConfig(kVision.BLU.NAME, kVision.BLU.ROBOT_TO_CAMERA),
            new PVManager.CameraConfig(kVision.YEL.NAME, kVision.YEL.ROBOT_TO_CAMERA));

    // -----------------------------
    // Shooter + Model
    // -----------------------------
    private final Supplier<Pose2d> poseSupplier = () -> drivetrain.getState().Pose;

    private final ShooterModel shooterModelConfig =
        ModelLoader.load("model.json", poseSupplier);

    public final ShooterSubsystem shooterBLUsystem =
        new ShooterSubsystem(
            kShooter.BLU_CONFIG,
            shooterModelConfig,
            poseSupplier,
            Hub.CENTER,
            logger,
            "Shooter-BLU");

    public final ShotTrainer shotBLUTrainer =
        new ShotTrainer(
            shooterBLUsystem.getShooterMotor(),
            poseSupplier,
            Hub.CENTER.getTranslation());

    public final ShooterSubsystem shooterYELsystem =
        new ShooterSubsystem(
            kShooter.YEL_CONFIG,
            shooterModelConfig,
            poseSupplier,
            Hub.CENTER,
            logger,
            "Shooter-YEL");

    public final ShotTrainer shotYELTrainer =
        new ShotTrainer(
            shooterYELsystem.getShooterMotor(),
            poseSupplier,
            Hub.CENTER.getTranslation());

    // -----------------------------
    // Feeder
    // -----------------------------
    public final FeederSubsystem feederBLUsystem =
        new FeederSubsystem(kFeeder.BLU_CONFIG);

    public final FeederSubsystem feederYELsystem =
        new FeederSubsystem(kFeeder.YEL_CONFIG);

    // -----------------------------
    // Intake
    // -----------------------------
    public final IntakeRollerSubsystem intakeRollerSubsystem =
        new IntakeRollerSubsystem(kIntake.ROLLER_CONFIG);

    public final IntakeArmSubsystem intakeArmSubsystem =
        new IntakeArmSubsystem(
            kIntake.kArm.BLU_CONFIG,
            kIntake.kArm.YEL_CONFIG);

    // -----------------------------
    // Agitator
    // -----------------------------
    public final AgitatorSubsystem agitatorSubsystem =
        new AgitatorSubsystem(kAgitator.CONFIG);

    // -----------------------------
    // Climber
    // -----------------------------
    public final ClimberSubsystem climberSubsystem =
        new ClimberSubsystem(
            kClimber.CONFIG,
            kClimber.SERVO_PORT);

    // -----------------------------
    // Hopper
    // -----------------------------
    public final HopperSubsystem hopperSubsystem =
        new HopperSubsystem(kHopper.CONFIG);

    // ----------------------------------------------------------
    // Recording Subsystems
    // ----------------------------------------------------------
    private final SubsystemsRecord subsystemsRecord = new SubsystemsRecord(
        drivetrain,
        agitatorSubsystem,
        climberSubsystem,
        feederBLUsystem,
        feederYELsystem,
        hopperSubsystem,
        intakeArmSubsystem,
        intakeRollerSubsystem,
        shooterBLUsystem,
        shooterYELsystem,
        PVManager,
        questNav);

    // ----------------------------------------------------------
    // Dashboard manager
    // ----------------------------------------------------------
    private final DashboardManager dashboardManager =
        new DashboardManager(
            subsystemsRecord,
            autoChooser);

    // ----------------------------------------------------------
    // Pre-created commands for whileTrue bindings
    // ----------------------------------------------------------
    private final Command maintainHeadingToTargetCommand =
        new CmdMaintainHeadingToTarget(
            drivetrain,
            shooterBLUsystem::getTargetPose,
            () -> -bottomDriver.getLeftY() * MaxSpeed,
            () -> -bottomDriver.getLeftX() * MaxSpeed);

    private final Command intakeDeployCommand =
        new CmdIntakeDeploy(intakeArmSubsystem, intakeRollerSubsystem);

    private final Command shootCommand =
        new CmdShoot(
            200, 200, 0.35,
            agitatorSubsystem,
            feederBLUsystem,
            feederYELsystem,
            shooterBLUsystem);

    // ----------------------------------------------------------
    // Robot Container Constructor
    // ----------------------------------------------------------
    public RobotContainer() {
        configureTelemetry();
        configureDefaultCommands();
        configureDriverControls();
        configureAutos();

        // Manager of all NT and Dashboard data
        dashboardManager.initDashboard();
    }

    // ----------------------------------------------------------
    // Default commands
    // ----------------------------------------------------------
    private void configureDefaultCommands() {

        drivetrain.setDefaultCommand(driveCommand);

        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true));
    }

    // ----------------------------------------------------------
    // Driver controls
    // ----------------------------------------------------------
    private void configureDriverControls() {

        // Drivetrain SysId
        bottomDriver.back().and(bottomDriver.y())
            .whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        bottomDriver.back().and(bottomDriver.x())
            .whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        bottomDriver.start().and(bottomDriver.y())
            .whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        bottomDriver.start().and(bottomDriver.x())
            .whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset field-centric heading
        bottomDriver.a()
            .onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        // Left bumper: slow mode while held, normal when released, reseed heading on press
        bottomDriver.leftBumper()
            .onTrue(Commands.runOnce(() -> {
                MaxSpeed = 0.3 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
                MaxAngularRate = RotationsPerSecond.of(0.50).in(RadiansPerSecond);
                drivetrain.seedFieldCentric();
            }));

        bottomDriver.leftBumper()
            .onFalse(Commands.runOnce(() -> {
                MaxSpeed = 0.65 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
                MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
            }));

        // Preplanned poses
        bottomDriver.povDown()
            .onTrue(new CmdMoveToPose(drivetrain, Tower.APPROACH_LEFT, MaxSpeed, MaxAngularRate));

        bottomDriver.povLeft()
            .onTrue(new CmdMoveToPose(drivetrain, Hub.APPROACH_LEFT, MaxSpeed, MaxAngularRate));

        bottomDriver.povRight()
            .onTrue(new CmdMoveToPose(drivetrain, Hub.APPROACH_RIGHT, MaxSpeed, MaxAngularRate));

        // Maintain heading to shooter target while driving
        bottomDriver.rightStick()
            .whileTrue(maintainHeadingToTargetCommand);

        // Intake
        bottomDriver.leftTrigger(0.5)
            .whileTrue(intakeDeployCommand);

        // Shooter + feeders + agitator
        bottomDriver.rightTrigger()
            .whileTrue(shootCommand);

        // Climber
        topDriver.y()
            .onTrue(new CmdClimberRobotUp(climberSubsystem));

        topDriver.x()
            .onTrue(new CmdClimberRobotDown(climberSubsystem));

        // Agitator
        topDriver.povUp()
            .whileTrue(new CmdAgitateToShooter(agitatorSubsystem));

        topDriver.povDown()
            .whileTrue(new CmdAgitateToIntake(agitatorSubsystem));
    }

    // ----------------------------------------------------------
    // Telemetry
    // ----------------------------------------------------------
    private void configureTelemetry() {
        logger.registerVisionPoseSource("PhotonVisionManager");
        logger.registerVisionPoseSource("Photon-BLU");
        logger.registerVisionPoseSource("Photon-YEL");

        drivetrain.registerTelemetry(logger::logDriveState);
    }

    // ----------------------------------------------------------
    // Autos
    // ----------------------------------------------------------
    private void configureAutos() {

        autoChooser.setDefaultOption(
            "Auto Do Nothing",
            Commands.print("Doing nothing"));

        autoChooser.addOption(
            "One Piece Auto",
            AutoSample.build(subsystemsRecord, MaxSpeed, MaxAngularRate));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    // ----------------------------------------------------------
    // Dashboard access for Robot.java
    // ----------------------------------------------------------
    public DashboardManager getDashboard() {
        return dashboardManager;
    }
}
