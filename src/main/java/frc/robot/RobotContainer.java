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
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;

// Commands
import frc.robot.commands.atomic.*;
import frc.robot.commands.auto.routines.*;
import frc.robot.commands.coordinators.*;

// Subsystems
import frc.robot.subsystems.*;

// Localization
import frc.robot.localization.LocalizationManager;
import frc.robot.localization.nodes.Nodes.Hub;
import frc.robot.localization.nodes.Nodes.Tower;
import frc.robot.localization.vision.PVManager;
import frc.robot.localization.vision.QuestNavManager;

// IO Layer (Hardware Abstractions)
import frc.lib.io.agitator.*;
import frc.lib.io.climber.*;
import frc.lib.io.feeder.*;
import frc.lib.io.hopper.*;
import frc.lib.io.intakearm.*;
import frc.lib.io.intakeroller.*;
import frc.lib.io.photonvision.*;
import frc.lib.io.shooter.*;
import frc.lib.io.swerve.*;

// Hardware Config / Vendor Libraries
import frc.robot.generated.ctre.CommandSwerveDrivetrain;
import frc.robot.generated.ctre.TunerConstants;
import frc.lib.hardware.*;

// Core
import frc.lib.core.logging.Telemetry;
import frc.lib.core.shooterML.data.*;
import frc.lib.core.shooterML.model.*;

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

    private Command defaultDriveCommand() {
        return swerve.run(() -> {
            double scale = bottomDriver.leftBumper().getAsBoolean() 
                ? kSwerve.kScale.CREEP 
                : kSwerve.kScale.NORMAL;

            double vx = -bottomDriver.getLeftY() * kSwerve.MAX_SPEED * scale;
            double vy = -bottomDriver.getLeftX() * kSwerve.MAX_SPEED * scale;
            double omega = -bottomDriver.getRightX() * kSwerve.MAX_ANGULAR_RATE * scale;

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
        new PhotonVisionIOReal(() -> swerve.getHeading());

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

    // -----------------------------
    // Shooter IO (Real or Sim)
    // -----------------------------
    private final ShooterIO shooterBLUIO =
        RobotBase.isReal()
            ? new ShooterIOReal(
                ShooterHardware.BLU_ID,
                kShooter.BLU_CONFIG,
                ShooterHardware.BLU_DIO
            )
            : new ShooterIOSim(
                kShooter.BLU_CONFIG,
                kShooter.kSim.FLYWHEEL_MODEL,
                ShooterHardware.BLU_RATIO
            );

    private final ShooterIO shooterYELIO =
        RobotBase.isReal()
            ? new ShooterIOReal(
                ShooterHardware.YEL_ID,
                kShooter.YEL_CONFIG,
                ShooterHardware.YEL_DIO
            )
            : new ShooterIOSim(
                kShooter.YEL_CONFIG,
                kShooter.kSim.FLYWHEEL_MODEL,
                ShooterHardware.YEL_RATIO
            );


    // pose + model
    // ----------------------------
    private final Supplier<Pose2d> poseSupplier = localizationManager::getFusedPose;

    private final ShooterModel shooterModelConfig =
        ModelLoader.load(
            Filesystem.getDeployDirectory().toPath().resolve("model.json").toString(), 
            poseSupplier);

    // subsystems (note: NO ShotTrainer arg)
    // ----------------------------
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

    // trainers (ShooterSubsystem implements ShooterTelemetryProvider)
    // ----------------------------
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
    // ----------------------------
    private final AgitatorSubsystem agitatorSubsystem =
        new AgitatorSubsystem(
            new AgitatorIOReal(AgitatorHardware.AGITATOR_ID, kAgitator.CONFIG));

    // Climber
    // ----------------------------
    public final ClimberSubsystem climberSubsystem =
        new ClimberSubsystem(
            new ClimberIOReal(
                ClimberHardware.CLIMBER_MOTOR_ID,
                kClimber.CONFIG_SLOT0, kClimber.CONFIG_SLOT1,
                ClimberHardware.RATIO,
                ClimberHardware.SERVO_PORT,
                ClimberHardware.LIMIT_SWITCH_PORT));

    // Feeder
    // ----------------------------
    public final FeederSubsystem feederBLUsystem =
        new FeederSubsystem(
            new FeederIOReal(FeederHardware.BLU_ID, kFeeder.BLU_CONFIG));  // BLUE


    public final FeederSubsystem feederYELsystem =
        new FeederSubsystem(
            new FeederIOReal(FeederHardware.YEL_ID, kFeeder.YEL_CONFIG)); // YELLOW

    // Hopper
    // ----------------------------
    public final HopperSubsystem hopperSubsystem =
        new HopperSubsystem(
            RobotBase.isReal()
                ? new HopperIOReal(
                    HopperHardware.HOPPER_ID,
                    kHopper.CONFIG,
                    HopperHardware.RATIO,
                    HopperHardware.HOPPER_DIO)
                : new HopperIOSim()
        );
  
    // Intake Arm
    // ----------------------------
    public final IntakeArmSubsystem intakeArmSubsystem =
        new IntakeArmSubsystem(
            RobotBase.isReal()
                ? new IntakeArmIOReal(
                    IntakeArmHardware.BLU_ID, kIntake.kArm.BLU_CONFIG, 
                    IntakeArmHardware.YEL_ID, kIntake.kArm.YEL_CONFIG,
                    IntakeArmHardware.RATIO)
                : new IntakeArmIOSim()
        );

    // Intake Roller
    // ----------------------------
    public final IntakeRollerSubsystem intakeRollerSubsystem =
        new IntakeRollerSubsystem(
            RobotBase.isReal()
                ? new IntakeRollerIOReal(IntakeRollerHardware.ROLLER_ID, kIntake.ROLLER_CONFIG)
                : new IntakeRollerIOSim()
        );

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
    // CoordinatorRecord
    // ==========================================================
    private final CoordinatorRecord coordinatorRecord =
        new CoordinatorRecord(
            shooterYELsystem, shooterBLUsystem, 
            feederYELsystem, feederBLUsystem, 
            agitatorSubsystem, intakeRollerSubsystem
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

    private final DashboardManagerMatch dashboardManagerMatch =
        new DashboardManagerMatch(subsystemsRecord, localizationRecord, autoChooser);

    // ==========================================================
    // Robot Container Constructor
    // ==========================================================
    public RobotContainer() {
        shooterBLUsystem.setShotTrainer(shotBLUTrainer);
        shooterYELsystem.setShotTrainer(shotYELTrainer);

        configureTelemetry();
        configureDefaultCommands();
        configureDriverControls();
        configureAutos();

        // Manager of all NT and Dashboard data
        dashboardManager.initDashboard();
        dashboardManagerMatch.initDashboard();
    }

    // ==========================================================
    // Default commands
    // ==========================================================
    private void configureDefaultCommands() {

        // Swerve Default Commands
        // ----------------------------
        swerve.setDefaultCommand(defaultDriveCommand());

        RobotModeTriggers.disabled()
            .whileTrue(
                Commands.run(swerve::stop, swerve).ignoringDisable(true)
            );

        // Intake Arm Default Command
        // ----------------------------
        intakeArmSubsystem.setDefaultCommand(
            IntakeArmCommands.manualPower(intakeArmSubsystem, () -> topDriver.getLeftY())
        );

        // Hopper Default Command
        // ----------------------------
        hopperSubsystem.setDefaultCommand(
         HopperCommands.manualPower(hopperSubsystem, () -> topDriver.getRightY())
        );
    }


    // ==========================================================
    // Driver controls
    // ==========================================================
    private void configureDriverControls() {

        // ----------------------------
        // Drive
        // ----------------------------

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

        bottomDriver.x()
            .whileTrue(
                DriveCommands.lock(swerve)
            );

        // ----------------------------
        // Preplanned poses
        // ----------------------------
        bottomDriver.povDown()
            .onTrue(DriveCommands.moveToPose(swerve, Tower.APPROACH_LEFT, kSwerve.MAX_SPEED, kSwerve.MAX_ANGULAR_RATE));

        bottomDriver.povLeft()
            .onTrue(DriveCommands.moveToPose(swerve, Hub.APPROACH_LEFT, kSwerve.MAX_SPEED, kSwerve.MAX_ANGULAR_RATE));

        bottomDriver.povRight()
            .onTrue(DriveCommands.moveToPose(swerve, Hub.APPROACH_RIGHT, kSwerve.MAX_SPEED, kSwerve.MAX_ANGULAR_RATE));

        // ----------------------------
        // Intake
        // ----------------------------

        topDriver.rightTrigger(0.5)
            .onTrue(IntakeRollerCommands.highRollerSpeed(intakeRollerSubsystem));

        topDriver.leftTrigger(0.5)
            .onTrue(IntakeRollerCommands.lowRollerSpeed(intakeRollerSubsystem));

        bottomDriver.leftTrigger(0.5)
            .whileTrue(
                IntakeSequences.deployAndRun(
                    hopperSubsystem,
                    intakeArmSubsystem,
                    intakeRollerSubsystem
                )
            )
            .onFalse(
                IntakeSequences.stow(
                    intakeArmSubsystem,
                    intakeRollerSubsystem
                )
            );

        bottomDriver.b()
            .whileTrue (IntakeRollerCommands.outtake(intakeRollerSubsystem))
            .whileTrue(IntakeArmCommands.down(intakeArmSubsystem))
            .onFalse(IntakeRollerCommands.stop(intakeRollerSubsystem))
            .onFalse(IntakeArmCommands.up(intakeArmSubsystem));
        
        // ----------------------------
        // Shooting
        // ----------------------------

        // shoot ML
          bottomDriver.rightTrigger()
            .whileTrue(ShooterControllers.shootModelBased(coordinatorRecord, kAgitator.AGITATE_TO_SHOOTER_DUTY, kIntake.INTAKE_ROLLER_DUTY_HIGH));

        // Shoot lob
        bottomDriver.rightBumper()
            .whileTrue(ShooterControllers.shootFixedRPM(coordinatorRecord, kShooter.kRPM.LOB, kAgitator.AGITATE_TO_SHOOTER_DUTY, kIntake.INTAKE_ROLLER_DUTY_HIGH));
        
        // ----------------------------
        // Climber
        //----------------------------

        //need to hold the button for the climber to move up or down
        topDriver.y().whileTrue(ClimberCommands.robotUp(climberSubsystem)); 
        topDriver.x().whileTrue(ClimberCommands.robotDown(climberSubsystem));

        // ----------------------------
        // Agitator
        // ----------------------------

        topDriver.povUp().whileTrue(AgitatorCommands.toShooter(agitatorSubsystem));
        topDriver.povDown().whileTrue(AgitatorCommands.toIntake(agitatorSubsystem));

        // ----------------------------
        // Reset Gyro
        // ----------------------------
        bottomDriver.start().whileTrue(
            Commands.run(() -> localizationManager.resetHeadingToZero())
        );
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
            "Auto Blue Subway Right",
            AutoSubwayRight.build(subsystemsRecord, coordinatorRecord, localizationManager::resetPose, kSwerve.MAX_SPEED * 0.8, kSwerve.MAX_ANGULAR_RATE));

        autoChooser.addOption(
            "Auto Blue Subway Left",
            AutoSubwayLeft.build(subsystemsRecord, coordinatorRecord, localizationManager::resetPose, kSwerve.MAX_SPEED * 0.5, kSwerve.MAX_ANGULAR_RATE));

        autoChooser.addOption(
            "Auto Shoot Until",
            AutoShootUntil.build(subsystemsRecord, coordinatorRecord, localizationManager::resetPose, kSwerve.MAX_SPEED * 0.5, kSwerve.MAX_ANGULAR_RATE));

        autoChooser.addOption(
            "Autoah Raymond",
            AutoahRaymond.build(subsystemsRecord, coordinatorRecord, localizationManager::resetPose, kSwerve.MAX_SPEED * 0.5, kSwerve.MAX_ANGULAR_RATE));

        autoChooser.addOption(
            "Auto Blue Human Player Q",
            AutoHumanPlayerQuest.build(subsystemsRecord, coordinatorRecord, localizationManager::resetPose, kSwerve.MAX_SPEED * 0.5, kSwerve.MAX_ANGULAR_RATE));

        autoChooser.addOption(
            "Auto Blue Depot",
            AutoDepot.build(subsystemsRecord, coordinatorRecord, localizationManager::resetPose, kSwerve.MAX_SPEED * 0.5, kSwerve.MAX_ANGULAR_RATE));

        autoChooser.addOption(
            "Auto Blue Quest Test",
            AutoTestQuest.build(subsystemsRecord, coordinatorRecord, localizationManager::resetPose, kSwerve.MAX_SPEED * 0.5, kSwerve.MAX_ANGULAR_RATE));

        autoChooser.addOption(
            "Auto Move Log",
            AutoMoveLog.build(subsystemsRecord, coordinatorRecord, localizationManager::resetPose, kSwerve.MAX_SPEED * 0.5, kSwerve.MAX_ANGULAR_RATE));
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
    public DashboardManagerMatch getDashboardMatch() {
        return dashboardManagerMatch;
    }
}
