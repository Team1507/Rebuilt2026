package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;

import frc.robot.subsystems.*;
import frc.robot.commands.agitate.CmdAgitatorManual;
import frc.robot.commands.climb.CmdClimberManual;
import frc.robot.commands.feed.CmdFeederManual;
import frc.robot.commands.intake.CmdIntakeArmManual;
import frc.robot.commands.intake.CmdIntakeRollerManual;
import frc.robot.commands.shoot.CmdShooterManual;
import frc.robot.commands.shoot.CmdShooterPIDTuner;
import frc.robot.Constants.kShooter;

import org.littletonrobotics.junction.Logger;

public class DashboardManager {

    // Subsystems
    private final CommandSwerveDrivetrain drivetrain;
    private final ShooterSubsystem shooterBLU;
    private final ShooterSubsystem shooterYEL;
    private final FeederSubsystem feederBLU;
    private final FeederSubsystem feederYEL;
    private final IntakeArmSubsystem intakeArm;
    private final IntakeRollerSubsystem intakeRoller;
    private final AgitatorSubsystem agitator;
    private final ClimberSubsystem climber;
    private final SendableChooser<?> autoChooser;

    // Manual values (read from NT)
    private double manualAgitatorDuty;
    private double manualClimberPosition;
    private double manualBLUFeederRPM;
    private double manualYELFeederRPM;
    private double manualIntakeArmAngle;
    private double manualIntakeRollerDuty;
    private double manualBLUShooterRPM;
    private double manualYELShooterRPM;
    private double shooterIdleRPM;

    // NT publishers for manual controls (Elastic UI)
    private final NetworkTable nt = NetworkTableInstance.getDefault().getTable("SmartDashboard");

    private final DoublePublisher pubAgitatorDuty =
        nt.getDoubleTopic("Manual Mode/Agitator/Target DC").publish();

    private final DoublePublisher pubClimberPos =
        nt.getDoubleTopic("Manual Mode/Climber/Target Position").publish();

    private final DoublePublisher pubBLUFeederRPM =
        nt.getDoubleTopic("Manual Mode/Feeder/BLU/Target RPM").publish();

    private final DoublePublisher pubYELFeederRPM =
        nt.getDoubleTopic("Manual Mode/Feeder/YEL/Target RPM").publish();

    private final DoublePublisher pubIntakeArmAngle =
        nt.getDoubleTopic("Manual Mode/Intake/Arm/Target Angle").publish();

    private final DoublePublisher pubIntakeRollerDuty =
        nt.getDoubleTopic("Manual Mode/Intake/Roller/Target DC").publish();

    private final DoublePublisher pubBLUShooterRPM =
        nt.getDoubleTopic("Manual Mode/Shooter/BLU/Terget RPM").publish();

    private final DoublePublisher pubYELShooterRPM =
        nt.getDoubleTopic("Manual Mode/Shooter/YEL/Terget RPM").publish();

    private final DoublePublisher pubShooterIdleRPM =
        nt.getDoubleTopic("Shooter/Shooter Idle RPM").publish();

    // Throttle
    private double lastUpdateTime = 0.0;
    private static final double DASHBOARD_PERIOD = 0.05; // 20 Hz

    public DashboardManager(
        CommandSwerveDrivetrain drivetrain,
        ShooterSubsystem shooterBLU,
        ShooterSubsystem shooterYEL,
        FeederSubsystem feederBLU,
        FeederSubsystem feederYEL,
        IntakeArmSubsystem intakeArm,
        IntakeRollerSubsystem intakeRoller,
        AgitatorSubsystem agitator,
        ClimberSubsystem climber,
        SendableChooser<?> autoChooser) {

        this.drivetrain = drivetrain;
        this.shooterBLU = shooterBLU;
        this.shooterYEL = shooterYEL;
        this.feederBLU = feederBLU;
        this.feederYEL = feederYEL;
        this.intakeArm = intakeArm;
        this.intakeRoller = intakeRoller;
        this.agitator = agitator;
        this.climber = climber;
        this.autoChooser = autoChooser;
    }

    // -------------------------------------------------
    // Initialization: SmartDashboard ONLY for buttons + chooser
    // -------------------------------------------------
    public void initDashboard() {

        // Auto chooser stays on SmartDashboard
        SmartDashboard.putData("Auto Mode", autoChooser);

        // Command buttons stay on SmartDashboard
        SmartDashboard.putData("Manual Mode/Agitator/Run",
            new CmdAgitatorManual(agitator, () -> manualAgitatorDuty));

        SmartDashboard.putData("Manual Mode/Climber/Run",
            new CmdClimberManual(climber, () -> manualClimberPosition));

        SmartDashboard.putData("Manual Mode/Feeder/BLU/Run",
            new CmdFeederManual(feederBLU, () -> manualBLUFeederRPM));

        SmartDashboard.putData("Manual Mode/Feeder/YEL/Run",
            new CmdFeederManual(feederYEL, () -> manualYELFeederRPM));

        SmartDashboard.putData("Manual Mode/Intake/Arm/Run",
            new CmdIntakeArmManual(intakeArm, () -> manualIntakeArmAngle));

        SmartDashboard.putData("Manual Mode/Intake/Roller/Run",
            new CmdIntakeRollerManual(intakeRoller, () -> manualIntakeRollerDuty));

        SmartDashboard.putData("Run Shooter PID Tuner",
            new CmdShooterPIDTuner(shooterBLU, kShooter.MAX_RPM));

        SmartDashboard.putData("Manual Mode/Shooter/BLU/Run",
            new CmdShooterManual(shooterBLU, () -> manualBLUShooterRPM));

        SmartDashboard.putData("Manual Mode/Shooter/YEL/Run",
            new CmdShooterManual(shooterYEL, () -> manualYELShooterRPM));
    }

    // -------------------------------------------------
    // Periodic update: read NT inputs + log telemetry
    // -------------------------------------------------
    public void updateInputs() {

        double now = Timer.getFPGATimestamp();
        if (now - lastUpdateTime < DASHBOARD_PERIOD) return;
        lastUpdateTime = now;

        // -----------------------------
        // Read manual control inputs (NT)
        // -----------------------------
        manualAgitatorDuty = nt.getEntry("Manual Mode/Agitator/Target DC").getDouble(manualAgitatorDuty);
        manualClimberPosition = nt.getEntry("Manual Mode/Climber/Target Position").getDouble(manualClimberPosition);
        manualBLUFeederRPM = nt.getEntry("Manual Mode/Feeder/BLU/Target RPM").getDouble(manualBLUFeederRPM);
        manualYELFeederRPM = nt.getEntry("Manual Mode/Feeder/YEL/Target RPM").getDouble(manualYELFeederRPM);
        manualIntakeArmAngle = nt.getEntry("Manual Mode/Intake/Arm/Target Angle").getDouble(manualIntakeArmAngle);
        manualIntakeRollerDuty = nt.getEntry("Manual Mode/Intake/Roller/Target DC").getDouble(manualIntakeRollerDuty);
        manualBLUShooterRPM = nt.getEntry("Manual Mode/Shooter/BLU/Terget RPM").getDouble(manualBLUShooterRPM);
        manualYELShooterRPM = nt.getEntry("Manual Mode/Shooter/YEL/Terget RPM").getDouble(manualYELShooterRPM);
        shooterIdleRPM = nt.getEntry("Shooter/Shooter Idle RPM").getDouble(shooterIdleRPM);

        // -----------------------------
        // Publish manual values back to NT (Elastic UI)
        // -----------------------------
        pubAgitatorDuty.set(manualAgitatorDuty);
        pubClimberPos.set(manualClimberPosition);
        pubBLUFeederRPM.set(manualBLUFeederRPM);
        pubYELFeederRPM.set(manualYELFeederRPM);
        pubIntakeArmAngle.set(manualIntakeArmAngle);
        pubIntakeRollerDuty.set(manualIntakeRollerDuty);
        pubBLUShooterRPM.set(manualBLUShooterRPM);
        pubYELShooterRPM.set(manualYELShooterRPM);
        pubShooterIdleRPM.set(shooterIdleRPM);

        // -----------------------------
        // AdvantageKit telemetry logging
        // -----------------------------
        Logger.recordOutput("Pigeon/HeadingDeg",
            drivetrain.getPigeon2().getRotation2d().getDegrees());

        Logger.recordOutput("Shooter/BLU/TargetRPM", shooterBLU.getTargetRPM());
        Logger.recordOutput("Shooter/BLU/ActualRPM", shooterBLU.getShooterRPM());
        Logger.recordOutput("Shooter/YEL/TargetRPM", shooterYEL.getTargetRPM());
        Logger.recordOutput("Shooter/YEL/ActualRPM", shooterYEL.getShooterRPM());

        Logger.recordOutput("Feeder/BLU/CurrentRPM", feederBLU.getVelocityRPM());
        Logger.recordOutput("Feeder/YEL/CurrentRPM", feederYEL.getVelocityRPM());

        Logger.recordOutput("Intake/Roller/DutyCycle", intakeRoller.getDutyCycle());
        Logger.recordOutput("Intake/Arm/BLU/AngleDeg", intakeArm.getBLUPositionDegrees());
        Logger.recordOutput("Intake/Arm/YEL/AngleDeg", intakeArm.getYELPositionDegrees());
    }

    public double getShooterIdleRPM() {
        return shooterIdleRPM;
    }
}
