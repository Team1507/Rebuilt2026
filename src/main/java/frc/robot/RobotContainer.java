//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

//reorganize imports later
package frc.robot;

import java.util.function.Supplier;

// CTRE Imports
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.revrobotics.servohub.ServoHub;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;

// WPI libraries
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import static edu.wpi.first.units.Units.*;

import frc.robot.commands.climb.CmdClimberRatchet;
import frc.robot.commands.climb.CmdClimberReset;
import frc.robot.commands.climb.CmdClimberRobotUp;
import frc.robot.commands.drive.CmdMoveToPose;
import frc.robot.commands.feed.CmdFeederFeed;
import frc.robot.commands.intake.CmdIntakeDeploy;
import frc.robot.commands.shoot.CmdShoot;
import frc.robot.commands.shoot.CmdShooterPIDTuner;
// Shooter
import frc.robot.shooter.data.ShotTrainer;
import frc.robot.shooter.model.ModelLoader;
import frc.robot.shooter.model.ShooterModel;

// Subsytem Imports
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.AgitatorSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.HopperSubsystem;

// Vision
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.utilities.SubsystemsRecord;
// Robot Extra
import frc.robot.utilities.Telemetry;
import frc.robot.navigation.Nodes.Hub;
import frc.robot.generated.TunerConstants;
import frc.robot.mechanics.GearRatio;
import frc.robot.Constants.kAgitator;

// Constants
import frc.robot.Constants.kFeeder;
import frc.robot.Constants.kIntake;
import frc.robot.Constants.kShooter;
import frc.robot.Constants.kVision;
import frc.robot.Constants.kClimber;
import frc.robot.Constants.kHopper;
import static frc.robot.Constants.kShooter.*;

// Autos
import frc.robot.auto.routines.AutoSample;


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
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    // Main swerve drivetrain instance generated from Phoenix Tuner configs
    public final CommandSwerveDrivetrain drivetrain =
        TunerConstants.createDrivetrain();

    // Telemetry logger for drivetrain and vision data
    private final Telemetry logger = new Telemetry(MaxSpeed);

    // Primary Xbox controller for driver input
    private final CommandXboxController joystick = new CommandXboxController(0);
private final CommandXboxController topstick = new CommandXboxController(1);
    // -----------------------------
    // Vision
    // -----------------------------

    public final Vision PVManager =
        new Vision(
            drivetrain::addVisionMeasurement,
            drivetrain::getHeading,          // Supplier<Rotation2d>
            drivetrain::seedPoseFromVision,
            new VisionIOPhotonVision(kVision.BLU.NAME, kVision.BLU.ROBOT_TO_CAMERA),
            new VisionIOPhotonVision(kVision.YEL.NAME, kVision.YEL.ROBOT_TO_CAMERA));

    // -----------------------------
    // Shooter + Model
    // -----------------------------
    private final GearRatio ratio = GearRatio.gearBox(1, 2);
    
    private final Supplier<Pose2d> poseSupplier = () -> drivetrain.getState().Pose;

    // Load model.json from deploy directory
    private final ShooterModel shooterModelConfig =
        ModelLoader.load("model.json", poseSupplier);

    public final ShooterSubsystem shooterBLUsystem =
        new ShooterSubsystem(
            new TalonFX(kShooter.BLU.CAN_ID),
            ratio,
            shooterModelConfig,
            poseSupplier,
            Hub.CENTER, // default target
            kShooter.BLU.ROBOT_TO_SHOOTER
        );

    //declare shooter RPM variable
    public double shooterRPM = 3000;
    public double shooterShootRPM = 5000;

    public final ShotTrainer shotBLUTrainer =
        new ShotTrainer(
            shooterBLUsystem.getShooterMotor(),
            poseSupplier,
            Hub.CENTER.getTranslation()
        );

    // -----------------------------
    // feeder
    // -----------------------------
    public final FeederSubsystem feederBLUsystem =
        new FeederSubsystem(
            kFeeder.BLU_CONFIG
        );

    public final FeederSubsystem feederYELsystem =
        new FeederSubsystem(
            kFeeder.YEL_CONFIG
        ); 
    private double feederTargetRPM = 500.0;

    // -----------------------------
    // intake
    // -----------------------------
    public final IntakeSubsystem intakeSubsystem =
        new IntakeSubsystem(
            kIntake.ROLLER_CONFIG
        );

    public final IntakeArmSubsystem intakeArmSubsystem =
        new IntakeArmSubsystem(
            kIntake.kArm.BLU_CONFIG,
            kIntake.kArm.YEL_CONFIG
        );

    // -----------------------------
    // Agitator
    // -----------------------------
     public final AgitatorSubsystem agitatorSubsystem =
        new AgitatorSubsystem(
            kAgitator.CONFIG
            
        );

    private double AgitatorTargetRPM = 500.0;

    // -----------------------------
    // climber
    // -----------------------------
    public final ClimberSubsystem climberSubsystem =
        new ClimberSubsystem(
            kClimber.CONFIG,
            kClimber.SERVO_PORT
        );

    // -----------------------------
    // hopper
    // -----------------------------
    public final HopperSubsystem hopperSubsystem =
        new HopperSubsystem(
            kHopper.CONFIG
        );

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
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        shooterBLUsystem.setDefaultCommand(
            Commands.run(
                () -> 
                    // Build telemetry → ask model → set RPM
                    //shooterSubsystem.updateShooterFromModel();
                    shooterBLUsystem.setTargetRPM(shooterRPM),
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
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        joystick.y().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));


        joystick.leftBumper().whileTrue(new InstantCommand(() -> {
            MaxSpeed = 0.2 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
            MaxAngularRate = RotationsPerSecond.of(0.50).in(RadiansPerSecond);
        }));
        joystick.leftBumper().whileFalse(new InstantCommand(() -> {
            MaxSpeed = 0.75 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
            MaxAngularRate = RotationsPerSecond.of(.75).in(RadiansPerSecond);
        }));
        joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
        joystick.b().whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.b().whileTrue(drivetrain.sysIdDynamic(Direction.kForward));

        // When button is pressed, set to full speed
        // true = button pressed
        joystick.leftBumper().onTrue(new InstantCommand(() -> {
            MaxSpeed = 0.3 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
            MaxAngularRate = RotationsPerSecond.of(0.50).in(RadiansPerSecond);
        }));

        // When button is released, go back to slow mode
        // false = button released
        joystick.leftBumper().onFalse(new InstantCommand(() -> {
            MaxSpeed = 0.65 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
            MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
        }));

        // ---------------------------------
        // Feeder
        // ---------------------------------

        joystick.b()
            .whileTrue(new CmdFeederFeed(feederTargetRPM, feederBLUsystem));
        joystick.b()
            .whileTrue(new CmdFeederFeed(feederTargetRPM, feederYELsystem));

        // ---------------------------------
        // Intake
        // ---------------------------------

        joystick.leftTrigger()
            .whileTrue(new CmdIntakeDeploy(intakeArmSubsystem, intakeSubsystem));

        // ---------------------------------
        // Shooter
        // ---------------------------------

        joystick.rightTrigger()
            .whileTrue(new CmdShoot(shooterShootRPM, 500, 100, agitatorSubsystem, feederYELsystem, feederBLUsystem, shooterBLUsystem));
        
            topstick.a().onTrue (new CmdClimberRatchet(climberSubsystem));
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
            intakeSubsystem,
            shooterBLUsystem

        );
        // Default auto
        autoChooser.setDefaultOption(
            "Auto Do Nothing",
            Commands.print("Doing nothing")
        );

        // --- Example autos using the new builder ---
        autoChooser.addOption(
            "One Piece Auto",
            AutoSample.build(record, MaxSpeed, MaxAngularRate)
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
    private void configureDashboard() {

        // ---------------------------------
        // Auto
        // ---------------------------------
        SmartDashboard.putData("Auto Mode", autoChooser);

        // ---------------------------------
        // Shooter
        // ---------------------------------
        SmartDashboard.putNumber("Shooter/Shooter Idle RPM", shooterRPM);
        SmartDashboard.putNumber("Shooter/ShooterShootRPM", shooterShootRPM);
        SmartDashboard.putData(
            "Run Shooter PID Tuner",
            new CmdShooterPIDTuner(shooterBLUsystem, MAX_RPM));

        SmartDashboard.putNumber("Shooter/BLU/TargetRPM", shooterBLUsystem.getTargetRPM());
        SmartDashboard.putNumber("Shooter/BLU/ActualRPM", shooterBLUsystem.getShooterRPM());
        SmartDashboard.putNumber("Shooter/BLU/TargetRPM", shooterRPM);
        SmartDashboard.putNumber("Shooter/BLU/Voltage", shooterBLUsystem.getShooterVoltage());
        SmartDashboard.putNumber("Shooter/BLU/StatorCurrent", shooterBLUsystem.getStatorCurrent());
        SmartDashboard.putNumber("Shooter/BLU/SupplyCurrent", shooterBLUsystem.getSupplyCurrent());
        SmartDashboard.putNumber("Shooter/BLU/ClosedLoopError", shooterBLUsystem.getClosedLoopError());

        // ---------------------------------
        // Feeder
        // ---------------------------------
        SmartDashboard.putNumber("Feeder/Target RPM", feederTargetRPM);

        // ---------------------------------
        // Intake
        // ---------------------------------
        SmartDashboard.putNumber("Intake/Arm/Blue/Angle", intakeArmSubsystem.getBLUPositionDegrees());
        SmartDashboard.putNumber("Intake/Arm/Yellow/Angle", intakeArmSubsystem.getYELPositionDegrees());
        // ---------------------------------
        // Agitator
        // ---------------------------------

        // ---------------------------------
        // Climber
        // ---------------------------------
        SmartDashboard.putNumber("Climber Position", climberSubsystem.getPosition());
        SmartDashboard.putData("Reset Climber", new CmdClimberReset(climberSubsystem));
        // ---------------------------------
        // Hooper
        // ---------------------------------

        // ---------------------------------
        // Vision
        // ---------------------------------
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

        // Read operator input
        shooterRPM = SmartDashboard.getNumber("Shooter RPM", shooterRPM);

        // Publish live telemetry
        SmartDashboard.putNumber(
            "Pigeon heading",
            drivetrain.getPigeon2().getRotation2d().getDegrees()
        );
    }
}
