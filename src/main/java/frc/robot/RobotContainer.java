// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//reorganize imports later
package frc.robot;

// CTRE Imports
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;

// WPI libraries
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// Commands
import frc.robot.commands.CmdFeederFeed;
import frc.robot.commands.CmdIntakeDeploy;
import frc.robot.commands.CmdShooterPIDTuner;
import frc.robot.commands.CmdShoot;

// Shooter
import frc.robot.shooter.data.PoseSupplier;
import frc.robot.shooter.data.ShotTrainer;
import frc.robot.shooter.model.ModelLoader;
import frc.robot.shooter.model.ShooterModel;

// Subsytem Imports
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.AgitatorSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;

// Robot Extra
import frc.robot.utilities.Telemetry;
import frc.robot.navigation.Nodes.AllianceZoneBlue;
import frc.robot.navigation.Nodes.Hub;
import frc.robot.generated.TunerConstants;
import frc.robot.mechanics.GearRatio;
import frc.robot.Constants.Agitator;
// Constants
import frc.robot.Constants.Feeder;
import frc.robot.Constants.Intake;
import frc.robot.Constants.Shooter;
import frc.robot.Constants.Vision;
import frc.robot.auto.routines.AutoSample;

import static frc.robot.Constants.Speed.*;
import static frc.robot.Constants.Shooter.*;

import static edu.wpi.first.units.Units.*;
public class RobotContainer {
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();
    
    private double MaxSpeed = 0.3 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final Telemetry logger = new Telemetry(getMaxSpeed());
    private final CommandXboxController joystick = new CommandXboxController(0);


    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    
    // -----------------------------
    // Cameras
    // -----------------------------

    public final PhotonVisionSubsystem visionBLUsystem = 
        new PhotonVisionSubsystem(
            drivetrain,
            logger,
            Vision.BLU.NANME,
            Vision.BLU.CAMERA_TO_ROBOT,
            Vision.BLU.PHOTONVISION_STD_DEVS
        );
    
    public final PhotonVisionSubsystem visionYELsystem = 
        new PhotonVisionSubsystem(
            drivetrain,
            logger,
            Vision.YEL.NANME,
            Vision.YEL.CAMERA_TO_ROBOT,
            Vision.YEL.PHOTONVISION_STD_DEVS
        );

    // -----------------------------
    // Shooter + Model
    // -----------------------------
    private final GearRatio ratio = GearRatio.gearBox(1, 2);
    
    private final PoseSupplier poseSupplier = () -> drivetrain.getState().Pose;

    // Load model.json from deploy directory
    private final ShooterModel shooterModelConfig =
        ModelLoader.load("model.json", poseSupplier);

    public final ShooterSubsystem shooterSubsystem =
        new ShooterSubsystem(
            new TalonFX(SHOOTER_CAN_ID),
            ratio,
            shooterModelConfig,
            poseSupplier,
            Hub.CENTER, // default target
            SHOOTER_OFFSET
        );

    //declare shooter RPM variable
    public double shooterRPM = 3000;
    public double shooterShootRPM = 5000;

    public final ShotTrainer shotTrainer =
        new ShotTrainer(
            shooterSubsystem.getShooterMotor(),
            poseSupplier,
            Hub.CENTER.getTranslation()
        );
    // -----------------------------
    //      feeder
    // -----------------------------
    public final FeederSubsystem feederSubsystem =
        new FeederSubsystem(
            new TalonFX(Feeder.FEEDER_CAN_ID),
            GearRatio.gearBox(1, 1)
        );
    private double feederTargetRPM = 500.0;

    public RobotContainer() {
        configureBindings();
        configureShooterDefault();
    }
    public double feederRPM;
    // -----------------------------
    //     intake?
    // -----------------------------
    public final IntakeSubsystem intakeSubsystem =
        new IntakeSubsystem(
            new TalonFX(Intake.INTAKE_ROLLER_CAN_ID)
        );

    public final IntakeArmSubsystem intakeArmSubsystem =
        new IntakeArmSubsystem(
            new TalonFX(Intake.INTAKE_ARM_CAN_ID)
        );

    // -----------------------------
    //     agitator!!!!
    // -----------------------------

    public final AgitatorSubsystem agitatorSubsystem = 
        new AgitatorSubsystem(
            new TalonFXS(Agitator.AGITATOR_CAN_ID)
        );
    public double agitatorRPM;
    /**
     * Shooter default behavior: use the trained model.json
     */
    private void configureShooterDefault() {

        shooterSubsystem.setDefaultCommand(
            Commands.run(
                () -> {
                    // Build telemetry → ask model → set RPM
                    //shooterSubsystem.updateShooterFromModel();
                    shooterSubsystem.setTargetRPM(shooterRPM);
                },
                shooterSubsystem
            )
        );
    }
    

    private void configureBindings() 
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

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
        

        // ---------------------------------
        // Feeder
        // ---------------------------------

        SmartDashboard.putNumber("Feeder/Target RPM", feederTargetRPM);
        joystick.b()
            .whileTrue(new CmdFeederFeed(feederTargetRPM, feederSubsystem));

        // ---------------------------------
        // Intake?
        // ---------------------------------

        joystick.a()
            .whileTrue(new CmdIntakeDeploy(intakeArmSubsystem, intakeSubsystem));

        // ---------------------------------
        // Shooter
        // ---------------------------------

        SmartDashboard.putNumber("Shooter/Shooter Idle RPM", shooterRPM);
        SmartDashboard.putNumber("Shooter/ShooterShootRPM", shooterShootRPM);
        joystick.y()
            .whileTrue(new CmdShoot(shooterShootRPM, 500, 100, agitatorSubsystem, feederSubsystem, shooterSubsystem));
        



        // PID Tuner
        SmartDashboard.putData( 
            "Run Shooter PID Tuner",
            new CmdShooterPIDTuner(shooterSubsystem, MAX_RPM) // max RPM here
            
        );
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }

    private double applyDeadband(double value, double deadband) {
        return (Math.abs(value) < deadband) ? 0.0 : value;
    }

    public void updateDashboard(){
        shooterRPM = SmartDashboard.getNumber("Shooter/Shooter Idle RPM", shooterRPM);
        shooterShootRPM = SmartDashboard.getNumber("Shooter/ShooterShootRPM", shooterShootRPM);
        SmartDashboard.putNumber("Feeder/Feeder RPM", feederSubsystem.getVelocityRPM());


        visionBLUsystem.addVisionMeasurementToDrivetrain();
        visionYELsystem.addVisionMeasurementToDrivetrain();

         // Default auto
        autoChooser.setDefaultOption(
            "Auto Do Nothing",
            Commands.print("Doing nothing")
        );
    
        // Example autos using your new builder
        autoChooser.addOption(
            "Auto Sample",
            AutoSample.build(drivetrain)
        );
    
        // Publish to dashboard
        SmartDashboard.putData("Auto Mode", autoChooser);
    }
}


