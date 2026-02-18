//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;

import frc.robot.commands.AgitatorCommands;
import frc.robot.commands.ClimberCommands;
import frc.robot.commands.FeederCommands;
import frc.robot.commands.HopperCommands;
import frc.robot.commands.IntakeArmCommands;
import frc.robot.commands.IntakeRollerCommands;
import frc.robot.commands.ShooterCommands;
import frc.robot.commands.tuning.CmdShooterPIDTuner;

import frc.robot.framework.*;
import frc.robot.Constants.kShooter;

public class DashboardManager {

    private final SubsystemsRecord subsystems;
    private final LocalizationRecord localization;
    private final SendableChooser<?> autoChooser;

    // Manual values
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

    // NT table
    private final NetworkTable nt =
        NetworkTableInstance.getDefault().getTable("SmartDashboard");

    /* ---------------- Publishers for manual values ---------------- */

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

    /* ---------------- Publishers for manual RUN buttons ---------------- */

    private final BooleanPublisher pubAgitatorRun =
        nt.getBooleanTopic("Manual Mode/Agitator/Run").publish();
    private final BooleanPublisher pubClimberRun =
        nt.getBooleanTopic("Manual Mode/Climber/Run").publish();
    private final BooleanPublisher pubBLUFeederRun =
        nt.getBooleanTopic("Manual Mode/Feeder/BLU/Run").publish();
    private final BooleanPublisher pubYELFeederRun =
        nt.getBooleanTopic("Manual Mode/Feeder/YEL/Run").publish();
    private final BooleanPublisher pubHopperRun =
        nt.getBooleanTopic("Manual Mode/Hopper/Run").publish();
    private final BooleanPublisher pubIntakeArmRun =
        nt.getBooleanTopic("Manual Mode/Intake/Arm/Run").publish();
    private final BooleanPublisher pubIntakeRollerRun =
        nt.getBooleanTopic("Manual Mode/Intake/Roller/Run").publish();
    private final BooleanPublisher pubBLUShooterRun =
        nt.getBooleanTopic("Manual Mode/Shooter/BLU/Run").publish();
    private final BooleanPublisher pubYELShooterRun =
        nt.getBooleanTopic("Manual Mode/Shooter/YEL/Run").publish();

    /* ---------------- Localization publishers ---------------- */

    private final BooleanPublisher pubLocalizationSeeded =
        nt.getBooleanTopic("Localization/PoseSeeded").publish();
    private final DoublePublisher pubQuestBattery =
        nt.getDoubleTopic("Localization/QuestNav/BatteryPercent").publish();
    private final BooleanPublisher pubQuestIsTracking =
        nt.getBooleanTopic("Localization/QuestNav/Is Tracking").publish();
    private final BooleanPublisher pubhasGoodVision =
        nt.getBooleanTopic("Localization/PhotonVision/Has Good Vision").publish();
        
    /* ---------------- Rising/Falling edge tracking ---------------- */

    private boolean prevAgitatorRun, prevClimberRun, prevBLUFeederRun, prevYELFeederRun;
    private boolean prevHopperRun, prevIntakeArmRun, prevIntakeRollerRun;
    private boolean prevBLUShooterRun, prevYELShooterRun;

    /* ---------------- Throttle ---------------- */

    private double lastUpdateTime = 0.0;
    private static final double DASHBOARD_PERIOD = 0.05; // 20 Hz

    public DashboardManager(
        SubsystemsRecord subsystems,
        LocalizationRecord localization,
        SendableChooser<?> autoChooser
    ) {
        this.subsystems = subsystems;
        this.localization = localization;
        this.autoChooser = autoChooser;

        // Initialize RUN buttons so Elastic sees them immediately
        pubAgitatorRun.set(false);
        pubClimberRun.set(false);
        pubBLUFeederRun.set(false);
        pubYELFeederRun.set(false);
        pubHopperRun.set(false);
        pubIntakeArmRun.set(false);
        pubIntakeRollerRun.set(false);
        pubBLUShooterRun.set(false);
        pubYELShooterRun.set(false);
    }

    /* ---------------- SmartDashboard init ---------------- */

    public void initDashboard() {
        SmartDashboard.putData("Auto Mode", autoChooser);

        SmartDashboard.putData("Run Blue Shooter PID Tuner",
            new CmdShooterPIDTuner(subsystems.BLUshooter(), kShooter.MAX_RPM));

        SmartDashboard.putData("Run Yellow Shooter PID Tuner",
            new CmdShooterPIDTuner(subsystems.YELshooter(), kShooter.MAX_RPM));
    }

    /* ---------------- Periodic update ---------------- */

    public void updateInputs() {

        double now = Timer.getFPGATimestamp();
        if (now - lastUpdateTime < DASHBOARD_PERIOD) return;
        lastUpdateTime = now;

        /* -------- Read manual values -------- */

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

        /* -------- Publish values back to Elastic -------- */

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

        pubLocalizationSeeded.set(localization.localizationManager().isStartupSeeded());
        pubQuestBattery.set(localization.questNav().getBatteryPercent().orElse(-1));
        pubQuestIsTracking.set(localization.questNav().isTracking());

        pubhasGoodVision.set(localization.pvManager().hasGoodVision());

        /* -------- Read RUN button states -------- */

        boolean agitatorRun = nt.getEntry("Manual Mode/Agitator/Run").getBoolean(false);
        boolean climberRun  = nt.getEntry("Manual Mode/Climber/Run").getBoolean(false);
        boolean bluFeederRun = nt.getEntry("Manual Mode/Feeder/BLU/Run").getBoolean(false);
        boolean yelFeederRun = nt.getEntry("Manual Mode/Feeder/YEL/Run").getBoolean(false);
        boolean hopperRun = nt.getEntry("Manual Mode/Hopper/Run").getBoolean(false);
        boolean intakeArmRun = nt.getEntry("Manual Mode/Intake/Arm/Run").getBoolean(false);
        boolean intakeRollerRun = nt.getEntry("Manual Mode/Intake/Roller/Run").getBoolean(false);
        boolean bluShooterRun = nt.getEntry("Manual Mode/Shooter/BLU/Run").getBoolean(false);
        boolean yelShooterRun = nt.getEntry("Manual Mode/Shooter/YEL/Run").getBoolean(false);

        /* -------- Rising-edge: schedule commands -------- */

        if (agitatorRun && !prevAgitatorRun)
            AgitatorCommands.manual(subsystems.agitator(), () -> manualAgitatorDuty).schedule();

        if (climberRun && !prevClimberRun)
            ClimberCommands.manual(subsystems.climber(), () -> manualClimberPosition).schedule();

        if (bluFeederRun && !prevBLUFeederRun)
            FeederCommands.manual(subsystems.BLUfeeder(), () -> manualBLUFeederRPM).schedule();

        if (yelFeederRun && !prevYELFeederRun)
            FeederCommands.manual(subsystems.YELfeeder(), () -> manualYELFeederRPM).schedule();

        if (hopperRun && !prevHopperRun)
            HopperCommands.manual(subsystems.hopper(), () -> manualHopperAngle).schedule();

        if (intakeArmRun && !prevIntakeArmRun)
            IntakeArmCommands.manual(subsystems.intakeArm(), () -> manualIntakeArmAngle).schedule();

        if (intakeRollerRun && !prevIntakeRollerRun)
            IntakeRollerCommands.manual(subsystems.intakeRoller(), () -> manualIntakeRollerDuty).schedule();

        if (bluShooterRun && !prevBLUShooterRun)
            ShooterCommands.manual(subsystems.BLUshooter(), () -> manualBLUShooterRPM).schedule();

        if (yelShooterRun && !prevYELShooterRun)
            ShooterCommands.manual(subsystems.YELshooter(), () -> manualYELShooterRPM).schedule();

        /* -------- Falling-edge: cancel commands -------- */

        if (!agitatorRun && prevAgitatorRun)
            subsystems.agitator().getCurrentCommand().cancel();

        if (!climberRun && prevClimberRun)
            subsystems.climber().getCurrentCommand().cancel();

        if (!bluFeederRun && prevBLUFeederRun)
            subsystems.BLUfeeder().getCurrentCommand().cancel();

        if (!yelFeederRun && prevYELFeederRun)
            subsystems.YELfeeder().getCurrentCommand().cancel();

        if (!hopperRun && prevHopperRun)
            subsystems.hopper().getCurrentCommand().cancel();

        if (!intakeArmRun && prevIntakeArmRun)
            subsystems.intakeArm().getCurrentCommand().cancel();

        if (!intakeRollerRun && prevIntakeRollerRun)
            subsystems.intakeRoller().getCurrentCommand().cancel();

        if (!bluShooterRun && prevBLUShooterRun)
            subsystems.BLUshooter().getCurrentCommand().cancel();

        if (!yelShooterRun && prevYELShooterRun)
            subsystems.YELshooter().getCurrentCommand().cancel();

        /* -------- Update previous states -------- */

        prevAgitatorRun = agitatorRun;
        prevClimberRun = climberRun;
        prevBLUFeederRun = bluFeederRun;
        prevYELFeederRun = yelFeederRun;
        prevHopperRun = hopperRun;
        prevIntakeArmRun = intakeArmRun;
        prevIntakeRollerRun = intakeRollerRun;
        prevBLUShooterRun = bluShooterRun;
        prevYELShooterRun = yelShooterRun;
    }

    public double getShooterIdleRPM() {
        return shooterIdleRPM;
    }
}
