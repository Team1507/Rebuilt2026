//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.robot;

// Java Standard Library
import java.util.function.Supplier;

// WPI / WPILib
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

import static edu.wpi.first.units.Units.*;

// Commands
import frc.robot.commands.*;
import frc.robot.commands.auto.routines.*;

// Subsystems
import frc.robot.subsystems.*;

// Localization
import frc.robot.localization.LocalizationManager;
import frc.robot.localization.nodes.Nodes.Hub;
import frc.robot.localization.nodes.Nodes.Tower;
import frc.robot.localization.vision.PVManager;
import frc.robot.localization.vision.QuestNavManager;

// IO Layer (Hardware Abstractions)
import frc.lib.io.agitator.AgitatorIOReal;
import frc.lib.io.climber.ClimberIOReal;
import frc.lib.io.feeder.FeederIOReal;
import frc.lib.io.hopper.HopperIOReal;
import frc.lib.io.intakearm.IntakeArmIOReal;
import frc.lib.io.intakeroller.IntakeRollerIOReal;
import frc.lib.io.photonvision.PhotonVisionIO;
import frc.lib.io.photonvision.PhotonVisionIOReal;
import frc.lib.io.shooter.*;
import frc.lib.io.swerve.*;

// Hardware Config / Vendor Libraries
import frc.robot.generated.ctre.CommandSwerveDrivetrain;
import frc.robot.generated.ctre.TunerConstants;
import frc.lib.hardware.ShooterHardware;

// Shooter ML Model
import frc.lib.shooterML.data.*;
import frc.lib.shooterML.model.*;
// Utilities
import frc.lib.logging.Telemetry;

// Framework
import frc.robot.framework.*;

// Constants
import frc.robot.Constants.*;

public class RobotContainer {

    // ==========================================================
    // Controllers
    // ==========================================================
    private final CommandXboxController bottomDriver = new CommandXboxController(0);
    private final CommandXboxController topDriver = new CommandXboxController(1);

    // ==========================================================
    // Telemetry Looger for all data being logged
    // ==========================================================
    private final Telemetry logger = new Telemetry();

    // ==========================================================
    // Drive configuration
    // ==========================================================

    private final CommandSwerveDrivetrain ctreDrivetrain =
        TunerConstants.createDrivetrain();

    private final SwerveSubsystem swerve =
        new SwerveSubsystem(new SwerveIOReal(ctreDrivetrain));

    private Command createDriveCommand() {
        return swerve.run(() -> {
            double vx = -bottomDriver.getLeftY() * kSwerve.MAX_SPEED;
            double vy = -bottomDriver.getLeftX() * kSwerve.MAX_SPEED;
            double omega = -bottomDriver.getRightX() * kSwerve.MAX_ANGULAR_RATE;

            swerve.drive(new ChassisSpeeds(vx, vy, omega));
        });
    }

    // ==========================================================
    // Localization
    // ==========================================================

    // QuestNav (IO-based, no longer fused directly)
    private final QuestNavManager questNav =
        new QuestNavManager(logger);

    // PhotonVision IO (explicit camera names)
    private final PhotonVisionIO pvIO =
        new PhotonVisionIOReal();

    // PhotonVision manager (fuses cameras only)
    private final PVManager pvManager =
        new PVManager(pvIO, logger);

    // Localization manager (final fusion layer)
    private final LocalizationManager localizationManager =
        new LocalizationManager(
            swerve,
            pvManager,
            questNav,
            logger
        );

    // ==========================================================
    // Shooter + Model
    // ==========================================================

    // Pose supplier for model-driven shooter + ShotTrainer
    //private final Supplier<Pose2d> poseSupplier = () -> ctreDrivetrain.getState().Pose;
    private final Supplier<Pose2d> poseSupplier = localizationManager::getFusedPose;

    // Load shooter model
    private final ShooterModel shooterModelConfig =
        ModelLoader.load("model.json", poseSupplier);

    // -------------------
    // Shooter IO (Real Hardware)
    // -------------------
    private final ShooterIO shooterBLUIO =
        new ShooterIOReal(
            ShooterHardware.BLU_ID,
            kShooter.BLU_CONFIG
        );

    private final ShooterIO shooterYELIO =
        new ShooterIOReal(
            ShooterHardware.YEL_ID,
            kShooter.YEL_CONFIG
        );

    // -------------------
    // Shooter Subsystems
    // -------------------
    public final ShooterSubsystem shooterBLUsystem =
        new ShooterSubsystem(
            shooterBLUIO,
            ShooterHardware.BLU_RATIO,
            ShooterHardware.ROBOT_TO_BLU_SHOOTER,
            shooterModelConfig,
            poseSupplier,
            Hub.CENTER,
            logger,
            "Shooter-BLU"
        );

    public final ShooterSubsystem shooterYELsystem =
        new ShooterSubsystem(
            shooterYELIO,
            ShooterHardware.YEL_RATIO,
            ShooterHardware.ROBOT_TO_YEL_SHOOTER,
            shooterModelConfig,
            poseSupplier,
            Hub.CENTER,
            logger,
            "Shooter-YEL"
        );

    // -------------------
    // Shot Trainers (IO-based)
    // -------------------
    public final ShotTrainer shotBLUTrainer =
        new ShotTrainer(
            shooterBLUsystem,
            poseSupplier,
            Hub.CENTER.getTranslation()
        );

    public final ShotTrainer shotYELTrainer =
        new ShotTrainer(
            shooterYELsystem,
            poseSupplier,
            Hub.CENTER.getTranslation()
        );

    // ==========================================================
    // Other subsystems
    // ==========================================================

    // Agitator
    private final AgitatorSubsystem agitatorSubsystem =
        new AgitatorSubsystem(
            new AgitatorIOReal(kAgitator.CONFIG));

    // Climber
    public final ClimberSubsystem climberSubsystem =
        new ClimberSubsystem(
            new ClimberIOReal(kClimber.CONFIG));

    // Feeder
    public final FeederSubsystem feederBLUsystem =
        new FeederSubsystem(
            new FeederIOReal(kFeeder.CONFIG, true));  // BLUE


    public final FeederSubsystem feederYELsystem =
        new FeederSubsystem(
            new FeederIOReal(kFeeder.CONFIG, false)); // YELLOW

    // Hopper
    public final HopperSubsystem hopperSubsystem =
        new HopperSubsystem(
            new HopperIOReal(kHopper.CONFIG));

    // Intake Arm
    public final IntakeArmSubsystem intakeArmSubsystem =
        new IntakeArmSubsystem(
            new IntakeArmIOReal(kIntake.kArm.BLU_CONFIG, kIntake.kArm.YEL_CONFIG));

    // Intake Roller
    public final IntakeRollerSubsystem intakeRollerSubsystem =
        new IntakeRollerSubsystem(
            new IntakeRollerIOReal(kIntake.ROLLER_CONFIG));

    // ==========================================================
    // SubsystemsRecord
    // ==========================================================
    private final SubsystemsRecord subsystemsRecord = 
        new SubsystemsRecord(
            swerve,
            agitatorSubsystem,
            climberSubsystem,
            feederBLUsystem,
            feederYELsystem,
            hopperSubsystem,
            intakeArmSubsystem,
            intakeRollerSubsystem,
            shooterBLUsystem,
            shooterYELsystem
        );

    // ==========================================================
    // LocalizationRecord
    // ==========================================================
    private final LocalizationRecord localizationRecord =
        new LocalizationRecord(
            localizationManager,
            pvManager,
            questNav
        );

    // ==========================================================
    // Autonomous chooser
    // ==========================================================
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    // ==========================================================
    // Dashboard manager
    // ==========================================================
    private final DashboardManager dashboardManager =
        new DashboardManager(subsystemsRecord, localizationRecord, autoChooser);

    // ==========================================================
    // Robot Container Constructor
    // ==========================================================
    public RobotContainer() {
        configureTelemetry();
        configureDefaultCommands();
        configureDriverControls();
        configureAutos();

        // Manager of all NT and Dashboard data
        dashboardManager.initDashboard();
    }

    // ==========================================================
    // Default commands
    // ==========================================================
    private void configureDefaultCommands() {

        swerve.setDefaultCommand(createDriveCommand());

        RobotModeTriggers.disabled().whileTrue(
            swerve.run(swerve::stop).ignoringDisable(true)
        );
    }

    // ==========================================================
    // Driver controls
    // ==========================================================
    private void configureDriverControls() {

        // Reset field-centric heading
        bottomDriver.a()
            .onTrue(ctreDrivetrain.runOnce(ctreDrivetrain::seedFieldCentric));

        // Left bumper: slow mode while held, normal when released, reseed heading on press
        bottomDriver.leftBumper()
            .onTrue(Commands.runOnce(() -> {
                kSwerve.MAX_SPEED = 0.3 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
                kSwerve.MAX_ANGULAR_RATE = RotationsPerSecond.of(0.50).in(RadiansPerSecond);
                ctreDrivetrain.seedFieldCentric();
            }));

        bottomDriver.leftBumper()
            .onFalse(Commands.runOnce(() -> {
                kSwerve.MAX_SPEED = 0.65 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
                kSwerve.MAX_ANGULAR_RATE = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
            }));

        // Preplanned poses
        bottomDriver.povDown()
            .onTrue(DriveCommands.moveToPose(swerve, Tower.APPROACH_LEFT, kSwerve.MAX_SPEED, kSwerve.MAX_ANGULAR_RATE));

        bottomDriver.povLeft()
            .onTrue(DriveCommands.moveToPose(swerve, Hub.APPROACH_LEFT, kSwerve.MAX_SPEED, kSwerve.MAX_ANGULAR_RATE));

        bottomDriver.povRight()
            .onTrue(DriveCommands.moveToPose(swerve, Hub.APPROACH_RIGHT, kSwerve.MAX_SPEED, kSwerve.MAX_ANGULAR_RATE));

        // Maintain heading to shooter target while driving
        bottomDriver.rightStick()
            .whileTrue(
                DriveCommands.maintainHeadingToTarget(
                    swerve,
                    shooterBLUsystem::getTargetPose,
                    () -> -bottomDriver.getLeftY() * kSwerve.MAX_SPEED,
                    () -> -bottomDriver.getLeftX() * kSwerve.MAX_SPEED
                )
            );

        // Intake
        bottomDriver.leftTrigger(0.5)
            .whileTrue(
                IntakeArmCommands.down(intakeArmSubsystem)
                    .alongWith(IntakeRollerCommands.intake(intakeRollerSubsystem))
            )
            .onFalse(
                IntakeArmCommands.up(intakeArmSubsystem)
                    .alongWith(IntakeRollerCommands.stop(intakeRollerSubsystem))
            );

        // Shooting
        bottomDriver.rightTrigger()
            .whileTrue(ShooterCoordinator.shootModelBased(
                shooterBLUsystem,
                shooterYELsystem,
                feederBLUsystem,
                feederYELsystem,
                agitatorSubsystem
            ));

        // Climber
        topDriver.y().onTrue(ClimberCommands.robotUp(climberSubsystem));
        topDriver.x().onTrue(ClimberCommands.robotDown(climberSubsystem));

        // Agitator
        topDriver.povUp().whileTrue(AgitatorCommands.toShooter(agitatorSubsystem));
        topDriver.povDown().whileTrue(AgitatorCommands.toIntake(agitatorSubsystem));

    }

    // ==========================================================
    // Telemetry
    // ==========================================================
    private void configureTelemetry() {
        ctreDrivetrain.registerTelemetry(logger::logDriveState);
    }

    // ==========================================================
    // Autos
    // ==========================================================
    private void configureAutos() {

        autoChooser.setDefaultOption("Do Nothing", Commands.print("Doing nothing"));

        autoChooser.addOption(
            "One Piece Auto",
            AutoSample.build(subsystemsRecord, kSwerve.MAX_SPEED, kSwerve.MAX_ANGULAR_RATE)
        );
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    // ==========================================================
    // Dashboard access for Robot.java
    // ==========================================================
    public DashboardManager getDashboard() {
        return dashboardManager;
    }
}
