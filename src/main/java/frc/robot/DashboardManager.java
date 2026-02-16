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
import frc.robot.commands.AgitatorCommands;
import frc.robot.commands.ClimberCommands;
import frc.robot.commands.FeederCommands;
import frc.robot.commands.HopperCommands;
import frc.robot.commands.IntakeArmCommands;
import frc.robot.commands.IntakeRollerCommands;
import frc.robot.commands.ShooterCommands;
import frc.robot.commands.tuning.CmdShooterPIDTuner;

// Robot Framework
import frc.robot.framework.*;

// Robot Constants
import frc.robot.Constants.kShooter;

public class DashboardManager {

    // Subsystems (via record)
    private final SubsystemsRecord subsystems;
    private final LocalizationRecord localization;
    private final SendableChooser<?> autoChooser;

    // Manual values (read from NT)
    private double manualAgitatorDuty;
    private double manualClimberPosition;
    private double manualBLUFeederRPM;
    private double manualYELFeederRPM;
    private double manualHopperAngle;
    private double manualIntakeArmAngle;
    private double manualIntakeRollerDuty;
    private double manualBLUShooterRPM;
    private double manualYELShooterRPM;
    private double shooterIdleRPM;

    // NT publishers for Elastic UI
    private final NetworkTable nt =
        NetworkTableInstance.getDefault().getTable("SmartDashboard");

    // Manual control publishers
    private final DoublePublisher pubAgitatorDuty =
        nt.getDoubleTopic("Manual Mode/Agitator/Target DC").publish();

    private final DoublePublisher pubClimberPos =
        nt.getDoubleTopic("Manual Mode/Climber/Target Position").publish();

    private final DoublePublisher pubBLUFeederRPM =
        nt.getDoubleTopic("Manual Mode/Feeder/BLU/Target RPM").publish();

    private final DoublePublisher pubYELFeederRPM =
        nt.getDoubleTopic("Manual Mode/Feeder/YEL/Target RPM").publish();

    private final DoublePublisher pubHopperAngle =
        nt.getDoubleTopic("Manual Mode/Hopper/Target Angle").publish();

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

    // Localization Publishers
    private final BooleanPublisher pubLocalizationSeeded =
        nt.getBooleanTopic("Localization/PoseSeeded").publish();

    private final DoublePublisher pubQuestBattery =
        nt.getDoubleTopic("Localization/QuestNav/BatteryPercent").publish();

    // Throttle
    private double lastUpdateTime = 0.0;
    private static final double DASHBOARD_PERIOD = 0.05; // 20 Hz

    public DashboardManager(
        SubsystemsRecord subsystems,
        LocalizationRecord localization,
        SendableChooser<?> autoChooser) {

        this.subsystems = subsystems;
        this.localization = localization;
        this.autoChooser = autoChooser;
    }

    // -------------------------------------------------
    // Initialization: SmartDashboard ONLY for buttons + chooser
    // -------------------------------------------------
    public void initDashboard() {

        // Auto chooser stays on SmartDashboard
        SmartDashboard.putData("Auto Mode", autoChooser);

        SmartDashboard.putData("Run Blue Shooter PID Tuner",
            new CmdShooterPIDTuner(subsystems.BLUshooter(), kShooter.MAX_RPM));

        SmartDashboard.putData("Run Yellow Shooter PID Tuner",
            new CmdShooterPIDTuner(subsystems.YELshooter(), kShooter.MAX_RPM));
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
        manualHopperAngle = nt.getEntry("Manual Mode/Hopper/Target Angle").getDouble(manualHopperAngle);
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
        pubHopperAngle.set(manualHopperAngle);
        pubIntakeArmAngle.set(manualIntakeArmAngle);
        pubIntakeRollerDuty.set(manualIntakeRollerDuty);
        pubBLUShooterRPM.set(manualBLUShooterRPM);
        pubYELShooterRPM.set(manualYELShooterRPM);
        pubShooterIdleRPM.set(shooterIdleRPM);

        // Publish Localization data to NT (Elastic UI)
        pubLocalizationSeeded.set(localization.localizationManager().isStartupSeeded());
        pubQuestBattery.set(localization.questNav().getBatteryPercent().orElse(-1));
    
        // -------------------------------------------------
        // Manual command triggers (Elastic UI buttons)
        // -------------------------------------------------
        if (nt.getEntry("Manual Mode/Agitator/Run").getBoolean(false))
            AgitatorCommands.manual(subsystems.agitator(), () -> manualAgitatorDuty).schedule();

        if (nt.getEntry("Manual Mode/Climber/Run").getBoolean(false))
            ClimberCommands.manual(subsystems.climber(), () -> manualClimberPosition).schedule();

        if (nt.getEntry("Manual Mode/Feeder/BLU/Run").getBoolean(false))
            FeederCommands.manual(subsystems.BLUfeeder(), () -> manualBLUFeederRPM).schedule();

        if (nt.getEntry("Manual Mode/Feeder/YEL/Run").getBoolean(false))
            FeederCommands.manual(subsystems.YELfeeder(), () -> manualYELFeederRPM).schedule();

        if (nt.getEntry("Manual Mode/Hopper/Run").getBoolean(false))
            HopperCommands.manual(subsystems.hopper(), () -> manualHopperAngle).schedule();

        if (nt.getEntry("Manual Mode/Intake/Arm/Run").getBoolean(false))
            IntakeArmCommands.manual(subsystems.intakeArm(), () -> manualIntakeArmAngle).schedule();

        if (nt.getEntry("Manual Mode/Intake/Roller/Run").getBoolean(false))
            IntakeRollerCommands.manual(subsystems.intakeRoller(), () -> manualIntakeRollerDuty).schedule();

        if (nt.getEntry("Manual Mode/Shooter/BLU/Run").getBoolean(false))
            ShooterCommands.manual(subsystems.BLUshooter(), () -> manualBLUShooterRPM).schedule();

        if (nt.getEntry("Manual Mode/Shooter/YEL/Run").getBoolean(false))
            ShooterCommands.manual(subsystems.YELshooter(), () -> manualYELShooterRPM).schedule();
    }

    public double getShooterIdleRPM() {
        return shooterIdleRPM;
    }
}
