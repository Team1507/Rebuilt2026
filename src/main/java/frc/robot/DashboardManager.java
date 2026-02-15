//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.robot;

// WPI Libraries
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;

// Robot Manual Commands
import frc.robot.commands.agitate.CmdAgitatorManual;
import frc.robot.commands.climb.CmdClimberManual;
import frc.robot.commands.feed.CmdFeederManual;
import frc.robot.commands.intake.CmdIntakeArmManual;
import frc.robot.commands.intake.CmdIntakeRollerManual;
import frc.robot.commands.shoot.CmdShooterManual;
import frc.robot.commands.shoot.CmdShooterPIDTuner;

// Utilities
import frc.robot.utilities.SubsystemsRecord;

// Robot Constants
import frc.robot.Constants.kShooter;

public class DashboardManager {

    // Subsystems (via record)
    private final SubsystemsRecord subsystems;
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

    // NT publishers for Elastic UI
    private final NetworkTable nt =
        NetworkTableInstance.getDefault().getTable("SmartDashboard");

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
        nt.getDoubleTopic("Manual Mode/Shooter/BLU/Target RPM").publish();

    private final DoublePublisher pubYELShooterRPM =
        nt.getDoubleTopic("Manual Mode/Shooter/YEL/Target RPM").publish();

    private final DoublePublisher pubShooterIdleRPM =
        nt.getDoubleTopic("Shooter/Shooter Idle RPM").publish();

    private final BooleanPublisher pubPVSeeded =
        nt.getBooleanTopic("Vision/PV/PoseSeeded").publish();

    private final DoublePublisher pubQuestBattery =
        nt.getDoubleTopic("Vision/QuestNav/BatteryPercent").publish();


    // Throttle
    private double lastUpdateTime = 0.0;
    private static final double DASHBOARD_PERIOD = 0.05; // 20 Hz

    public DashboardManager(
        SubsystemsRecord subsystems,
        SendableChooser<?> autoChooser) {

        this.subsystems = subsystems;
        this.autoChooser = autoChooser;
    }

    // -------------------------------------------------
    // Initialization: SmartDashboard ONLY for buttons + chooser
    // -------------------------------------------------
    public void initDashboard() {

        // Auto chooser stays on SmartDashboard
        SmartDashboard.putData("Auto Mode", autoChooser);

        // Command buttons (SmartDashboard only)
        SmartDashboard.putData("Manual Mode/Agitator/Run",
            new CmdAgitatorManual(subsystems.agitator(), () -> manualAgitatorDuty));

        SmartDashboard.putData("Manual Mode/Climber/Run",
            new CmdClimberManual(subsystems.climber(), () -> manualClimberPosition));

        SmartDashboard.putData("Manual Mode/Feeder/BLU/Run",
            new CmdFeederManual(subsystems.BLUfeeder(), () -> manualBLUFeederRPM));

        SmartDashboard.putData("Manual Mode/Feeder/YEL/Run",
            new CmdFeederManual(subsystems.YELfeeder(), () -> manualYELFeederRPM));

        SmartDashboard.putData("Manual Mode/Intake/Arm/Run",
            new CmdIntakeArmManual(subsystems.intakeArm(), () -> manualIntakeArmAngle));

        SmartDashboard.putData("Manual Mode/Intake/Roller/Run",
            new CmdIntakeRollerManual(subsystems.intakeRoller(), () -> manualIntakeRollerDuty));

        SmartDashboard.putData("Run Blue Shooter PID Tuner",
            new CmdShooterPIDTuner(subsystems.BLUshooter(), kShooter.MAX_RPM));

        SmartDashboard.putData("Manual Mode/Shooter/BLU/Run",
            new CmdShooterManual(subsystems.BLUshooter(), () -> manualBLUShooterRPM));

        SmartDashboard.putData("Run Yellow Shooter PID Tuner",
            new CmdShooterPIDTuner(subsystems.YELshooter(), kShooter.MAX_RPM));

        SmartDashboard.putData("Manual Mode/Shooter/YEL/Run",
            new CmdShooterManual(subsystems.YELshooter(), () -> manualYELShooterRPM));
    }

    // -------------------------------------------------
    // Periodic update: read NT inputs + publish UI values
    // -------------------------------------------------
    public void updateInputs() {

        double now = Timer.getFPGATimestamp();
        if (now - lastUpdateTime < DASHBOARD_PERIOD) return;
        lastUpdateTime = now;

        // Read manual control inputs (NT)
        manualAgitatorDuty = nt.getEntry("Manual Mode/Agitator/Target DC").getDouble(manualAgitatorDuty);
        manualClimberPosition = nt.getEntry("Manual Mode/Climber/Target Position").getDouble(manualClimberPosition);
        manualBLUFeederRPM = nt.getEntry("Manual Mode/Feeder/BLU/Target RPM").getDouble(manualBLUFeederRPM);
        manualYELFeederRPM = nt.getEntry("Manual Mode/Feeder/YEL/Target RPM").getDouble(manualYELFeederRPM);
        manualIntakeArmAngle = nt.getEntry("Manual Mode/Intake/Arm/Target Angle").getDouble(manualIntakeArmAngle);
        manualIntakeRollerDuty = nt.getEntry("Manual Mode/Intake/Roller/Target DC").getDouble(manualIntakeRollerDuty);
        manualBLUShooterRPM = nt.getEntry("Manual Mode/Shooter/BLU/Target RPM").getDouble(manualBLUShooterRPM);
        manualYELShooterRPM = nt.getEntry("Manual Mode/Shooter/YEL/Target RPM").getDouble(manualYELShooterRPM);
        shooterIdleRPM = nt.getEntry("Shooter/Shooter Idle RPM").getDouble(shooterIdleRPM);

        // Publish manual values back to NT (Elastic UI)
        pubAgitatorDuty.set(manualAgitatorDuty);
        pubClimberPos.set(manualClimberPosition);
        pubBLUFeederRPM.set(manualBLUFeederRPM);
        pubYELFeederRPM.set(manualYELFeederRPM);
        pubIntakeArmAngle.set(manualIntakeArmAngle);
        pubIntakeRollerDuty.set(manualIntakeRollerDuty);
        pubBLUShooterRPM.set(manualBLUShooterRPM);
        pubYELShooterRPM.set(manualYELShooterRPM);
        pubShooterIdleRPM.set(shooterIdleRPM);

        // Publish Vision data to NT (Elastic UI)
        pubPVSeeded.set(subsystems.pvManager().isVisionSeeded());
        pubQuestBattery.set(subsystems.questNav().getBatteryPercent().orElse(-1));
    }

    public double getShooterIdleRPM() {
        return shooterIdleRPM;
    }
}
