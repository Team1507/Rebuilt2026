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
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.*;
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

// Shooter Model
import frc.robot.shooter.data.*;
import frc.robot.shooter.model.*;
// Subsytem Imports
import frc.robot.subsystems.*;
import frc.robot.subsystems.vision.*;

// Robot Utilities
import frc.robot.utilities.*;
import frc.robot.navigation.Nodes.Hub;
import frc.robot.navigation.Nodes.Tower;
import frc.robot.generated.*;

// Constants
import frc.robot.Constants.*;

// Autos
import frc.robot.auto.routines.*;


public class RobotContainer {

    // Chooser for selecting autonomous routines on the dashboard
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    // Max robot drive speed (scaled from the drivetrain's 12V free speed)
    public static double MaxSpeed =
        0.5 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

    // Max rotational speed for teleop turning
    public static double MaxAngularRate =
        RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    // Base field‑centric drive request with deadbands and open‑loop voltage control
    private final SwerveRequest.FieldCentric drive =
        new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1)
            .withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);

    // Main swerve drivetrain instance generated from Phoenix Tuner configs
    public final CommandSwerveDrivetrain drivetrain =
        TunerConstants.createDrivetrain();

    // Telemetry logger for drivetrain and vision data
    private final Telemetry logger = new Telemetry(MaxSpeed);

    // Primary Xbox controller for driver input
    private final CommandXboxController bottomDriver = new CommandXboxController(0);
    private final CommandXboxController topDriver = new CommandXboxController(1);

    private boolean manualMode = false;
    // -----------------------------
    // Vision
    // -----------------------------

   private final QuestNavSubsystem questNav = new QuestNavSubsystem(drivetrain);

   public final Vision PVManager =
       new Vision(
           drivetrain::addVisionMeasurement,
            drivetrain::getHeading,          // Supplier<Rotation2d>
            drivetrain::seedPoseFromVision,
            questNav::setQuestNavPose,
            new Vision.CameraConfig(kVision.BLU.NAME, kVision.BLU.ROBOT_TO_CAMERA),
            new Vision.CameraConfig(kVision.YEL.NAME, kVision.YEL.ROBOT_TO_CAMERA));

    // -----------------------------
    // Shooter + Model
    // -----------------------------    
    private final Supplier<Pose2d> poseSupplier = () -> drivetrain.getState().Pose;

    // Load model.json from deploy directory
    private final ShooterModel shooterModelConfig =
        ModelLoader.load("model.json", poseSupplier);

    //declare shooter RPM variable
    public double shooterIdleRPM;
    public double shooterShootRPM;

    public double manualBLUShooterRPM;
    public double manualYELShooterRPM;

    public final ShooterSubsystem shooterBLUsystem =
        new ShooterSubsystem(
            kShooter.BLU_CONFIG,
            shooterModelConfig,
            poseSupplier,
            Hub.CENTER);

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
            Hub.CENTER);

    public final ShotTrainer shotYELTrainer =
        new ShotTrainer(
            shooterYELsystem.getShooterMotor(),
            poseSupplier,
            Hub.CENTER.getTranslation());

    // -----------------------------
    // feeder
    // -----------------------------
    public final FeederSubsystem feederBLUsystem =
        new FeederSubsystem(
            kFeeder.BLU_CONFIG);

    public final FeederSubsystem feederYELsystem =
        new FeederSubsystem(
            kFeeder.YEL_CONFIG); 
    
    public double manualBLUFeederRPM;
    public double manualYELFeederRPM;

    // -----------------------------
    // intake
    // -----------------------------
    public final IntakeRollerSubsystem intakeRollerSubsystem =
        new IntakeRollerSubsystem(
            kIntake.ROLLER_CONFIG);

    public double manualIntakeRollerDuty;

    public final IntakeArmSubsystem intakeArmSubsystem =
        new IntakeArmSubsystem(
            kIntake.kArm.BLU_CONFIG,
            kIntake.kArm.YEL_CONFIG);

    public double manualIntakeArmAngle;

    // -----------------------------
    // Agitator
    // -----------------------------
    public final AgitatorSubsystem agitatorSubsystem =
        new AgitatorSubsystem(
            kAgitator.CONFIG);

    public double manualAgitatorDuty;

    // -----------------------------
    // climber
    // -----------------------------
    public final ClimberSubsystem climberSubsystem =
        new ClimberSubsystem(
            kClimber.CONFIG,
            kClimber.SERVO_PORT);

    public double manualClimberPosition;

    // -----------------------------
    // hopper
    // -----------------------------
    public final HopperSubsystem hopperSubsystem =
        new HopperSubsystem(
            kHopper.CONFIG);

    /**
     * Constructs the {@code RobotContainer}, the central configuration hub for the robot.
     * <p>
     * This initializes all subsystems, loads shooter models, sets up default commands,
     * binds driver controls, configures vision processing, registers telemetry sources,
     * builds autonomous routines, and publishes key values to SmartDashboard.
     * <p>
     * The constructor ensures that all robot systems are fully wired together before
     * entering any robot mode, providing a single point of orchestration for runtime behavior.
     */
    public RobotContainer() {
        configureTelemetry();
        configureDefaultCommands();
        configureDriverControls();
        configureVision();
        configureAutos();
        configureDashboard();
    }

    /**
     * Configures the default commands for major subsystems.
     * <p>
     * This establishes the continuous behaviors that run whenever no other
     * commands are scheduled for a subsystem. For the drivetrain, this binds
     * joystick input to field‑centric swerve control. For the shooter, this
     * applies the idle RPM target so the flywheel maintains readiness.
     * <p>
     * Also registers an idle request during robot disable to ensure neutral
     * modes are applied consistently.
     */
    private void configureDefaultCommands() 
    {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-bottomDriver.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-bottomDriver.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-bottomDriver.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true));

        
        shooterBLUsystem.setDefaultCommand(
            Commands.run(
                () -> 
                    // Build telemetry → ask model → set RPM
                    //shooterSubsystem.updateShooterFromModel();
                    shooterBLUsystem.setTargetRPM(shooterIdleRPM),
                shooterBLUsystem
            )
        );
    }

    /**
     * Binds all driver and operator controller inputs to their respective actions.
     * <p>
     * This includes:
     * <ul>
     *   <li>SysId routines for drivetrain characterization</li>
     *   <li>Heading reset and drive‑mode scaling</li>
     *   <li>Feeder, intake, shooter, and agitator command bindings</li>
     * </ul>
     * The method centralizes all human‑interface logic so that control mappings
     * remain easy to audit and modify.
     */
    private void configureDriverControls() {

        // ---------------------------------
        // Drivetrain
        // ---------------------------------

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        bottomDriver.back().and(bottomDriver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        bottomDriver.back().and(bottomDriver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        bottomDriver.start().and(bottomDriver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        bottomDriver.start().and(bottomDriver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        bottomDriver.a().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));


        bottomDriver.leftBumper().whileTrue(new InstantCommand(() -> {
            MaxSpeed = 0.2 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
            MaxAngularRate = RotationsPerSecond.of(0.50).in(RadiansPerSecond);
        }));

        bottomDriver.leftBumper().whileFalse(new InstantCommand(() -> {
            MaxSpeed = 0.75 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
            MaxAngularRate = RotationsPerSecond.of(.75).in(RadiansPerSecond);
        }));

        bottomDriver.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
        bottomDriver.b().whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        bottomDriver.b().whileTrue(drivetrain.sysIdDynamic(Direction.kForward));

        // When button is pressed, set to full speed
        // true = button pressed
        bottomDriver.leftBumper().onTrue(new InstantCommand(() -> {
            MaxSpeed = 0.3 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
            MaxAngularRate = RotationsPerSecond.of(0.50).in(RadiansPerSecond);
        }));

        // When button is released, go back to slow mode
        // false = button released
        bottomDriver.leftBumper().onFalse(new InstantCommand(() -> {
            MaxSpeed = 0.65 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
            MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
        }));

        bottomDriver.povDown()
            .onTrue(new CmdMoveToPose(drivetrain, Tower.APPROACH_LEFT, MaxSpeed, MaxAngularRate));
        
        bottomDriver.povLeft()
            .onTrue(new CmdMoveToPose(drivetrain, Hub.APPROACH_LEFT, MaxSpeed, MaxAngularRate));

        bottomDriver.povRight()
            .onTrue(new CmdMoveToPose(drivetrain, Hub.APPROACH_RIGHT, MaxSpeed, MaxAngularRate));

        bottomDriver.rightStick().whileTrue(new CmdMaintainHeadingToTarget(
            drivetrain,
            () -> shooterBLUsystem.getTargetPose(), // Target pose supplier
            () -> -bottomDriver.getLeftY() * MaxSpeed, // X velocity supplier
            () -> -bottomDriver.getLeftX() * MaxSpeed  // Y velocity supplier
        ));
        // ---------------------------------
        // Feeder
        // ---------------------------------

        

        // ---------------------------------
        // Intake
        // ---------------------------------

        bottomDriver.leftTrigger(0.5)
            .whileTrue(new CmdIntakeDeploy(intakeArmSubsystem, intakeRollerSubsystem));

        // ---------------------------------
        // Shooter
        // ---------------------------------

        bottomDriver.rightTrigger()
            .whileTrue(new CmdShoot(200, 200, .35, agitatorSubsystem, feederBLUsystem, feederYELsystem, shooterBLUsystem));
        
        // ---------------------------------
        // Climber
        // ---------------------------------

            topDriver.y()
                .onTrue(new CmdClimberRobotUp(climberSubsystem));

            topDriver.x()
                .onTrue(new CmdClimberRobotDown(climberSubsystem));

        // ---------------------------------
        // Agitator
        // ---------------------------------

            topDriver.povUp()
                .whileTrue(new CmdAgitateToShooter(agitatorSubsystem));

            topDriver.povDown()
                .whileTrue(new CmdAgitateToIntake(agitatorSubsystem));
    }

    /**
     * Performs vision‑system configuration and initialization.
     * <p>
     * Currently a placeholder for future operator bindings or debug toggles.
     * Vision pipelines and pose‑estimation integration are initialized in the
     * constructor; this method exists to keep the structure consistent and
     * maintain a dedicated expansion point for vision‑related controls.
     */
    private void configureVision() {
        // Nothing to bind yet — vision runs autonomously
        // Future: buttons for resetVisionSeeding(), debug toggles, etc.
    }

    /**
     * Registers telemetry sources and attaches them to the drivetrain logger.
     * <p>
     * This includes PhotonVision pose sources and the drivetrain’s internal
     * telemetry stream. By centralizing telemetry registration here, the robot
     * ensures consistent logging across all modes and simplifies debugging and
     * performance analysis.
     */
    private void configureTelemetry() {
        logger.registerVisionPoseSource("PhotonVisionManager");
        logger.registerVisionPoseSource("Photon-BLU");
        logger.registerVisionPoseSource("Photon-YEL");

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    /**
     * Builds and registers autonomous routines with the SmartDashboard chooser.
     * <p>
     * Sets a default “do nothing” auto and adds additional routines constructed
     * through the project’s auto builder. This method ensures that all autos are
     * discoverable and selectable before the robot enters autonomous mode.
     */
    private void configureAutos() {
        SubsystemsRecord record = new SubsystemsRecord(
            drivetrain,
            agitatorSubsystem,
            climberSubsystem,
            feederBLUsystem,
            feederYELsystem,
            hopperSubsystem,
            intakeArmSubsystem,
            intakeRollerSubsystem,
            shooterBLUsystem
        );

        // Default auto
        autoChooser.setDefaultOption(
            "Auto Do Nothing",
            Commands.print("Doing nothing")
        );

        // --- Example autos using the new builder ---
        autoChooser.addOption(
            "Auto Subway Right",
            AutoBlueSubwayRight.build(record, MaxSpeed, MaxAngularRate)
        );

        autoChooser.addOption(
            "Auto Subway Left",
            AutoBlueSubwayLeft.build(record, MaxSpeed, MaxAngularRate)
        );

        // Publish to dashboard
        SmartDashboard.putData("Auto Mode", autoChooser);
    }    

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    /**
     * Publishes operator‑tunable parameters, subsystem controls, and diagnostic
     * values to SmartDashboard.
     * <p>
     * This includes shooter RPM setpoints, feeder targets, and command buttons
     * such as the Shooter PID Tuner. Dashboard values allow real‑time adjustment
     * and monitoring during development and testing.
     */
    public void configureDashboard() {

        // -----------------------------
        // Manual Mode Toggle
        // -----------------------------
        SmartDashboard.putBoolean("Manual Mode Enabled", manualMode);

        // -----------------------------
        // Auto
        // -----------------------------
        SmartDashboard.putData("Auto Mode", autoChooser);

        // -----------------------------
        // Manual Agitator Inputs (editable)
        // -----------------------------
        SmartDashboard.putNumber("Manual Mode/Agitator/Target DC", 0.0);

        SmartDashboard.putData(
            "Manual Mode/Agitator/Run",
            new CmdAgitatorManual(
                agitatorSubsystem,
                () -> manualAgitatorDuty
            )
        );

        // -----------------------------
        // Manual Climber Inputs (editable)
        // -----------------------------
        SmartDashboard.putNumber("Manual Mode/Climber/Target Position", 0.0);

        SmartDashboard.putData(
            "Manual Mode/Climber/Run",
            new CmdClimberManual(
                climberSubsystem,
                () -> manualClimberPosition
            )
        );

        // -----------------------------
        // Manual Feeder Inputs (editable)
        // -----------------------------
        SmartDashboard.putNumber("Manual Mode/Feeder/BLU/Target RPM", 0.0);

        SmartDashboard.putData(
            "Manual Mode/Feeder/BLU/Run",
            new CmdFeederManual(
                feederBLUsystem,
                () -> manualBLUFeederRPM
            )
        );

        SmartDashboard.putNumber("Manual Mode/Feeder/YEL/Target RPM", 0.0);

        SmartDashboard.putData(
            "Manual Mode/Feeder/YEL/Run",
            new CmdFeederManual(
                feederYELsystem,
                () -> manualYELFeederRPM
            )
        );
        
        // -----------------------------
        // Manual Intake Inputs (editable)
        // -----------------------------
        SmartDashboard.putNumber("Manual Mode/Intake/Arm/Target Angle", 0.0);

        SmartDashboard.putData(
            "Manual Mode/Intake/Arm/Run",
            new CmdIntakeArmManual(
                intakeArmSubsystem,
                () -> manualIntakeArmAngle
            )
        );

        SmartDashboard.putNumber("Manual Mode/Intake/Roller/Terget DC", 0.0);

        SmartDashboard.putData(
            "Manual Mode/Intake/Roller/Run",
            new CmdIntakeRollerManual(
                intakeRollerSubsystem,
                () -> manualIntakeRollerDuty
            )
        );

        // -----------------------------
        // Shooter Inputs (editable)
        // -----------------------------
        SmartDashboard.putNumber("Shooter/Shooter Idle RPM", 0.0);
        SmartDashboard.putNumber("Shooter/ShooterShootRPM", 0.0);

        SmartDashboard.putData(
            "Run Shooter PID Tuner",
            new CmdShooterPIDTuner(shooterBLUsystem, kShooter.MAX_RPM)
        );

        SmartDashboard.putNumber("Manual Mode/Shooter/BLU/Terget RPM", 0.0);

        SmartDashboard.putData(
            "Manual Mode/Shooter/BLU/Run",
            new CmdShooterManual(
                shooterBLUsystem,
                () -> manualBLUShooterRPM
            )
        );

        SmartDashboard.putNumber("Manual Mode/Shooter/YEL/Terget RPM", 0.0);

        SmartDashboard.putData(
            "Manual Mode/Shooter/YEL/Run",
            new CmdShooterManual(
                shooterBLUsystem,
                () -> manualYELShooterRPM
            )
        );
    }


    /**
     * Reads operator‑adjustable dashboard inputs and publishes live telemetry.
     * <p>
     * This method is intended to be called periodically (e.g., in robotPeriodic)
     * to synchronize SmartDashboard values with internal state. It updates shooter
     * RPM setpoints and reports drivetrain heading for debugging and driver
     * awareness.
     */
    public void updateDashboardInputs() {

        // -----------------------------
        // Read Operator Inputs
        // -----------------------------

        manualAgitatorDuty =
            SmartDashboard.getNumber(
                "Manual Mode/Agitator/Target DC",
                manualAgitatorDuty
            );

        manualClimberPosition =
            SmartDashboard.getNumber(
                "Manual Mode/Climber/Target Position",
                manualClimberPosition
            );

        manualBLUFeederRPM =
            SmartDashboard.getNumber(
                "Manual Mode/Feeder/BLU/Target RPM",
                manualBLUFeederRPM
            );

        manualYELFeederRPM =
            SmartDashboard.getNumber(
                "Manual Mode/Feeder/YEL/Target RPM",
                manualYELFeederRPM
            );

        manualIntakeArmAngle =
            SmartDashboard.getNumber(
                "Manual Mode/Intake/Arm/Target Angle",
                manualIntakeArmAngle
            );

        manualIntakeRollerDuty =
            SmartDashboard.getNumber(
                "Manual Mode/Intake/Roller/Target DC",
                manualIntakeRollerDuty
            );

        manualBLUShooterRPM =
            SmartDashboard.getNumber(
                "Manual Mode/Shooter/BLU/Terget RPM",
                manualBLUShooterRPM
            );

        manualYELShooterRPM =
            SmartDashboard.getNumber(
                "Manual Mode/Shooter/YEL/Terget RPM",
                manualYELShooterRPM
            );

        shooterIdleRPM =
            SmartDashboard.getNumber(
                "Shooter/Shooter Idle RPM",
                shooterIdleRPM
            );

        // -----------------------------
        // Telemetry (robot → dashboard)
        // -----------------------------

        SmartDashboard.putNumber(
            "Pigeon Heading",
            drivetrain.getPigeon2().getRotation2d().getDegrees()
        );

        // Shooter Telemetry
        SmartDashboard.putNumber("Shooter/BLU/TargetRPM", shooterBLUsystem.getTargetRPM());
        SmartDashboard.putNumber("Shooter/BLU/ActualRPM", shooterBLUsystem.getShooterRPM());
        SmartDashboard.putNumber("Shooter/YEL/TargetRPM", shooterYELsystem.getTargetRPM());
        SmartDashboard.putNumber("Shooter/YEL/ActualRPM", shooterYELsystem.getShooterRPM());

        // Feeder Telemetry
        SmartDashboard.putNumber("Feeder/BLU/Current RPM", feederBLUsystem.getVelocityRPM());
        SmartDashboard.putNumber("Feeder/YEL/Current RPM", feederYELsystem.getVelocityRPM());

        // Intake Telemetry
        SmartDashboard.putNumber("Intake/Roller/Duty Cycle", intakeRollerSubsystem.getDutyCycle());
        SmartDashboard.putNumber("Intake/Arm/BLU/Angle", intakeArmSubsystem.getBLUPositionDegrees());
        SmartDashboard.putNumber("Intake/Arm/YEL/Angle", intakeArmSubsystem.getYELPositionDegrees());
    }

}
