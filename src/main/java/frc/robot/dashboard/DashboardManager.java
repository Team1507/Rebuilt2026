//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.robot.dashboard;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.atomic.*;
import frc.robot.framework.*;
import frc.robot.localization.nodes.Nodes;

public class DashboardManager {

    private static final boolean ENABLE_MATCH   = true;
    private static final boolean ENABLE_MANUAL  = false;
    private static final boolean ENABLE_MONITOR = true;

    private final SubsystemsRecord   subsystems;
    private final LocalizationRecord localization;
    private final SendableChooser<?> autoChooser;

    private NetworkTable matchNT;
    private NetworkTable manualNT;
    private NetworkTable monitorNT;

    // Table presence publishers
    private BooleanPublisher pubManualEnabled;
    private BooleanPublisher pubMatchEnabled;
    private BooleanPublisher pubMonitorEnabled;

    /* ---------------- Throttle ---------------- */
    private double lastUpdate = 0.0;
    private static final double PERIOD = 0.20;   // 5 Hz

    /* ---------------- Manul ---------------- */
    private DoubleSubscriber  subAgitatorDuty;
    private DoubleSubscriber  subBLUFeederRPM;
    private DoubleSubscriber  subYELFeederRPM;
    private DoubleSubscriber  subHopperAngle;
    private DoubleSubscriber  subIntakeArmAngle;
    private DoubleSubscriber  subIntakeRollerDuty;
    private DoubleSubscriber  subBLUShooterRPM;
    private DoubleSubscriber  subYELShooterRPM;

    private BooleanSubscriber subAgitatorRun;
    private BooleanSubscriber subBLUFeederRun;
    private BooleanSubscriber subYELFeederRun;
    private BooleanSubscriber subHopperRun;
    private BooleanSubscriber subIntakeArmRun;
    private BooleanSubscriber subIntakeRollerRun;
    private BooleanSubscriber subBLUShooterRun;
    private BooleanSubscriber subYELShooterRun;

    private boolean prevAgitator, prevBLUFeeder, prevYELFeeder;
    private boolean prevHopper, prevIntakeArm, prevIntakeRoller;
    private boolean prevBLUShooter, prevYELShooter;

    /* ---------------- Match ---------------- */
    // Blue shooter
    private DoublePublisher   pubBLUShooterRPM;
    private DoublePublisher   pubBLUShooterTargetRPM;

    // Yellow shooter
    private DoublePublisher   pubYELShooterRPM;
    private DoublePublisher   pubYELShooterTargetRPM;

    // Hopper
    private DoublePublisher   pubHopperPos;
    private BooleanPublisher  pubHopperExtended;

    // Intake
    private DoublePublisher   pubIntakeBLUPos;
    private DoublePublisher   pubIntakeYELPos;
    private DoublePublisher   pubIntakeRollerCurrentDC;
    private DoublePublisher   pubIntakeRollerCmdDC;

    // Vision
    private BooleanPublisher  pubQuestNavConnected;
    private BooleanPublisher  pubQuestNavIsTracking;
    private IntegerPublisher  pubQuestNavBattery;

    private BooleanPublisher  pubSeedLeft;
    private BooleanPublisher  pubSeedCenter;
    private BooleanPublisher  pubSeedRight;

    private DoublePublisher   pubManualX;
    private DoublePublisher   pubManualY;
    private DoublePublisher   pubManualTheta;
    private BooleanPublisher  pubManualSeedPose;

    private BooleanPublisher  pubQuestNavIsResetComplete;

    /* ---------------- Monitor ---------------- */
    // Agitator
    private DoublePublisher   pubMonAgitatorDuty;
    private DoublePublisher   pubMonAgitatorTemp;
    private DoublePublisher   pubMonAgitatorCurrent;
    private BooleanPublisher  pubMonAgitatorFeeding;

    // Blue Feeder
    private DoublePublisher   pubMonBlueFeederRPM;
    private DoublePublisher   pubMonBlueFeederRPS;
    private DoublePublisher   pubMonBlueFeederTemp;
    private DoublePublisher   pubMonBlueFeederCurrent;

    // Yellow Feeder
    private DoublePublisher   pubMonYellowFeederRPM;
    private DoublePublisher   pubMonYellowFeederRPS;
    private DoublePublisher   pubMonYellowFeederTemp;
    private DoublePublisher   pubMonYellowFeederCurrent;

    // Hopper
    private DoublePublisher   pubMonHopperPosition;
    private BooleanPublisher  pubMonHopperMagSensor;
    private DoublePublisher   pubMonHopperTemp;
    private DoublePublisher   pubMonHopperCurrent;
    private BooleanPublisher  pubMonHopperRetracted;
    private BooleanPublisher  pubMonHopperExtended;
    private BooleanPublisher  pubMonHopperReverseLimit;
    
    // Blue Intake Arm
    private DoublePublisher   pubMonBlueIntakeArmPosition;
    private BooleanPublisher  pubMonBlueIntakeArmReverseLimit;
    private DoublePublisher   pubMonBlueIntakeArmMotorPosition;
    private DoublePublisher   pubMonBlueIntakeArmTemp;
    private DoublePublisher   pubMonBlueIntakeArmCurrent;
    private DoublePublisher   pubMonBlueIntakeArmVelocityDegPerSec;
    private DoublePublisher   pubMonBlueIntakeArmAppliedVolts;

    // Yellow Intake Arm
    private DoublePublisher   pubMonYellowIntakeArmPosition;
    private BooleanPublisher  pubMonYellowIntakeArmReverseLimit;
    private DoublePublisher   pubMonYellowIntakeArmMotorPosition;
    private DoublePublisher   pubMonYellowIntakeArmTemp;
    private DoublePublisher   pubMonYellowIntakeArmCurrent;
    private DoublePublisher   pubMonYellowIntakeArmVelocityDegPerSec;
    private DoublePublisher   pubMonYellowIntakeArmAppliedVolts;
    
    // Intake Roller
    private DoublePublisher   pubMonIntakeRollerDutyCycle;
    private DoublePublisher   pubMonIntakeRollerTemp;
    private DoublePublisher   pubMonIntakeRollerCurrent;
    private DoublePublisher   pubMonIntakeRollerCmdDutyCycle;

    // Blue Shooter
    private DoublePublisher   pubMonBlueShooterMotorRPS;
    private DoublePublisher   pubMonBlueShooterCurrentRPM;
    private DoublePublisher   pubMonBlueShooterTargetRPM;
    private DoublePublisher   pubMonBlueShooterDistanceToTarget;
    private DoublePublisher   pubMonBlueShooterTemp;
    private DoublePublisher   pubMonBlueShooterCurrent;

    // Yellow Shooter
    private DoublePublisher   pubMonYellowShooterMotorRPS;
    private DoublePublisher   pubMonYellowShooterCurrentRPM;
    private DoublePublisher   pubMonYellowShooterTargetRPM;
    private DoublePublisher   pubMonYellowShooterDistanceToTarget;
    private DoublePublisher   pubMonYellowShooterTemp;
    private DoublePublisher   pubMonYellowShooterCurrent;

    private static double round(double value, int decimals) {
        double scale = Math.pow(10, decimals);
        return Math.round(value * scale) / scale;
    }

    private static double whole(double value) {
        return Math.round(value);
    }

    public DashboardManager(
        SubsystemsRecord subsystems,
        LocalizationRecord localization,
        SendableChooser<?> autoChooser
    ) {
        this.subsystems = subsystems;
        this.localization = localization;
        this.autoChooser = autoChooser;

        if (ENABLE_MATCH) {
            matchNT = NetworkTableInstance.getDefault().getTable("Match");

            pubMatchEnabled =
                matchNT.getBooleanTopic("Enabled").publish();
            pubMatchEnabled.set(true);

            pubBLUShooterRPM =
                matchNT.getDoubleTopic("Shooter/BLU/Current RPM").publish();
            pubBLUShooterTargetRPM =
                matchNT.getDoubleTopic("Shooter/BLU/Target RPM").publish();

            pubYELShooterRPM =
                matchNT.getDoubleTopic("Shooter/YEL/Current RPM").publish();
            pubYELShooterTargetRPM =
                matchNT.getDoubleTopic("Shooter/YEL/Target RPM").publish();

            pubHopperPos =
                matchNT.getDoubleTopic("Hopper/CurrentPosition").publish();
            pubHopperExtended =
                matchNT.getBooleanTopic("Hopper/IsExtended").publish();

            pubIntakeBLUPos =
                matchNT.getDoubleTopic("Intake/Arm/BLU/Current Position").publish();
            pubIntakeYELPos =
                matchNT.getDoubleTopic("Intake/Arm/YEL/Current Position").publish();

            pubIntakeRollerCurrentDC =
                matchNT.getDoubleTopic("Intake/Roller/Roller Current DC").publish();
            pubIntakeRollerCmdDC =
                matchNT.getDoubleTopic("Intake/Roller/Roller Cmd DC").publish();

            pubQuestNavConnected =
                matchNT.getBooleanTopic("Localization/QuestNav/Connected").publish();
            pubQuestNavIsTracking =
                matchNT.getBooleanTopic("Localization/QuestNav/Is Tracking").publish();
            pubQuestNavBattery =
                matchNT.getIntegerTopic("Localization/QuestNav/Battery Percent").publish();
            pubQuestNavIsResetComplete =
                matchNT.getBooleanTopic("Localization/QuestNav/Is Reset Complete").publish();

            pubSeedLeft = 
                matchNT.getBooleanTopic("Localization/QuestNav/SeedLeft").publish();
            pubSeedCenter = 
                matchNT.getBooleanTopic("Localization/QuestNav/SeedCenter").publish();
            pubSeedRight = 
                matchNT.getBooleanTopic("Localization/QuestNav/SeedRight").publish();

            pubManualX = 
                matchNT.getDoubleTopic("Localization/QuestNav/ManualX").publish();
            pubManualY = 
                matchNT.getDoubleTopic("Localization/QuestNav/ManualY").publish();
            pubManualTheta = 
                matchNT.getDoubleTopic("Localization/QuestNav/ManualThetaDeg").publish();
            pubManualSeedPose = 
                matchNT.getBooleanTopic("Localization/QuestNav/ManualSeedPose").publish();
        }

        if (ENABLE_MANUAL) {
            manualNT = NetworkTableInstance.getDefault().getTable("Manual Mode");

            /* ---------- Table Presence ---------- */
            pubManualEnabled =
                manualNT.getBooleanTopic("Enabled").publish();
            pubManualEnabled.set(true);

            /* ---------- Seed Publishers (Layout Only) ---------- */
            manualNT.getDoubleTopic("Agitator/Target DC").publish().set(0.0);
            manualNT.getDoubleTopic("Feeder/BLU/Target RPM").publish().set(0.0);
            manualNT.getDoubleTopic("Feeder/YEL/Target RPM").publish().set(0.0);
            manualNT.getDoubleTopic("Hopper/Target Position").publish().set(0.0);
            manualNT.getDoubleTopic("Intake/Arm/Target Angle").publish().set(0.0);
            manualNT.getDoubleTopic("Intake/Roller/Target DC").publish().set(0.0);
            manualNT.getDoubleTopic("Shooter/BLU/Target RPM").publish().set(0.0);
            manualNT.getDoubleTopic("Shooter/YEL/Target RPM").publish().set(0.0);

            manualNT.getBooleanTopic("Agitator/Run").publish().set(false);
            manualNT.getBooleanTopic("Feeder/BLU/Run").publish().set(false);
            manualNT.getBooleanTopic("Feeder/YEL/Run").publish().set(false);
            manualNT.getBooleanTopic("Hopper/Run").publish().set(false);
            manualNT.getBooleanTopic("Intake/Arm/Run").publish().set(false);
            manualNT.getBooleanTopic("Intake/Roller/Run").publish().set(false);
            manualNT.getBooleanTopic("Shooter/BLU/Run").publish().set(false);
            manualNT.getBooleanTopic("Shooter/YEL/Run").publish().set(false);

            /* ---------- Subscribers (Actual Control Path) ---------- */
            subAgitatorDuty =
                manualNT.getDoubleTopic("Agitator/Target DC").subscribe(0.0);
            subBLUFeederRPM =
                manualNT.getDoubleTopic("Feeder/BLU/Target RPM").subscribe(0.0);
            subYELFeederRPM =
                manualNT.getDoubleTopic("Feeder/YEL/Target RPM").subscribe(0.0);
            subHopperAngle =
                manualNT.getDoubleTopic("Hopper/Target Position").subscribe(0.0);
            subIntakeArmAngle =
                manualNT.getDoubleTopic("Intake/Arm/Target Angle").subscribe(0.0);
            subIntakeRollerDuty =
                manualNT.getDoubleTopic("Intake/Roller/Target DC").subscribe(0.0);
            subBLUShooterRPM =
                manualNT.getDoubleTopic("Shooter/BLU/Target RPM").subscribe(0.0);
            subYELShooterRPM =
                manualNT.getDoubleTopic("Shooter/YEL/Target RPM").subscribe(0.0);

            subAgitatorRun =
                manualNT.getBooleanTopic("Agitator/Run").subscribe(false);
            subBLUFeederRun =
                manualNT.getBooleanTopic("Feeder/BLU/Run").subscribe(false);
            subYELFeederRun =
                manualNT.getBooleanTopic("Feeder/YEL/Run").subscribe(false);
            subHopperRun =
                manualNT.getBooleanTopic("Hopper/Run").subscribe(false);
            subIntakeArmRun =
                manualNT.getBooleanTopic("Intake/Arm/Run").subscribe(false);
            subIntakeRollerRun =
                manualNT.getBooleanTopic("Intake/Roller/Run").subscribe(false);
            subBLUShooterRun =
                manualNT.getBooleanTopic("Shooter/BLU/Run").subscribe(false);
            subYELShooterRun =
                manualNT.getBooleanTopic("Shooter/YEL/Run").subscribe(false);
        }

        if (ENABLE_MONITOR) {
            monitorNT = NetworkTableInstance.getDefault().getTable("Monitor");

            pubMonitorEnabled =
                monitorNT.getBooleanTopic("Enabled").publish();
            pubMonitorEnabled.set(true);

            // Agitator
            pubMonAgitatorDuty =
                monitorNT.getDoubleTopic("Agitator/DutyCycle").publish();
            pubMonAgitatorTemp =
                monitorNT.getDoubleTopic("Agitator/TemperatureC").publish();
            pubMonAgitatorCurrent =
                monitorNT.getDoubleTopic("Agitator/CurrentA").publish();
            pubMonAgitatorFeeding =
                monitorNT.getBooleanTopic("Agitator/FeedingEnabled").publish();

            // Feeder Blue
            pubMonBlueFeederRPM =
                monitorNT.getDoubleTopic("Feeder/BLU/VelocityRPM").publish();
            pubMonBlueFeederRPS =
                monitorNT.getDoubleTopic("Feeder/BLU/MotorRPS").publish();
            pubMonBlueFeederTemp =
                monitorNT.getDoubleTopic("Feeder/BLU/TemperatureC").publish();
            pubMonBlueFeederCurrent =
                monitorNT.getDoubleTopic("Feeder/BLU/CurrentA").publish();

            // Feeder Yellow
            pubMonYellowFeederRPM =
                monitorNT.getDoubleTopic("Feeder/YEL/VelocityRPM").publish();
            pubMonYellowFeederRPS =
                monitorNT.getDoubleTopic("Feeder/YEL/MotorRPS").publish();
            pubMonYellowFeederTemp =
                monitorNT.getDoubleTopic("Feeder/YEL/TemperatureC").publish();
            pubMonYellowFeederCurrent =
                monitorNT.getDoubleTopic("Feeder/YEL/CurrentA").publish();

            // Hopper
            pubMonHopperPosition =
                monitorNT.getDoubleTopic("Hopper/Position").publish();
            pubMonHopperMagSensor =
                monitorNT.getBooleanTopic("Hopper/MagSensor").publish();
            pubMonHopperTemp =
                monitorNT.getDoubleTopic("Hopper/TemperatureC").publish();
            pubMonHopperCurrent =
                monitorNT.getDoubleTopic("Hopper/CurrentA").publish();
            pubMonHopperRetracted =
                monitorNT.getBooleanTopic("Hopper/Retracted").publish();
            pubMonHopperExtended =
                monitorNT.getBooleanTopic("Hopper/Extended").publish();
            pubMonHopperReverseLimit =
                monitorNT.getBooleanTopic("Hopper/ReverseLimit").publish();

            // Intake Arm Blue
            pubMonBlueIntakeArmPosition =
                monitorNT.getDoubleTopic("IntakeArm/BLU/PositionDeg").publish();
            pubMonBlueIntakeArmMotorPosition =
                monitorNT.getDoubleTopic("IntakeArm/BLU/MotorRot").publish();
            pubMonBlueIntakeArmTemp =
                monitorNT.getDoubleTopic("IntakeArm/BLU/TemperatureC").publish();
            pubMonBlueIntakeArmCurrent =
                monitorNT.getDoubleTopic("IntakeArm/BLU/CurrentA").publish();
            pubMonBlueIntakeArmReverseLimit =
                monitorNT.getBooleanTopic("IntakeArm/BLU/ReverseLimit").publish();
            pubMonBlueIntakeArmVelocityDegPerSec =
                monitorNT.getDoubleTopic("IntakeArm/BLU/VelocityDegPerSec").publish();
            pubMonBlueIntakeArmAppliedVolts =
                monitorNT.getDoubleTopic("IntakeArm/BLU/AppliedVolts").publish();

            // Intake Arm Yellow
            pubMonYellowIntakeArmPosition =
                monitorNT.getDoubleTopic("IntakeArm/YEL/PositionDeg").publish();
            pubMonYellowIntakeArmMotorPosition =
                monitorNT.getDoubleTopic("IntakeArm/YEL/MotorRot").publish();
            pubMonYellowIntakeArmTemp =
                monitorNT.getDoubleTopic("IntakeArm/YEL/TemperatureC").publish();
            pubMonYellowIntakeArmCurrent =
                monitorNT.getDoubleTopic("IntakeArm/YEL/CurrentA").publish();
            pubMonYellowIntakeArmReverseLimit =
                monitorNT.getBooleanTopic("IntakeArm/YEL/ReverseLimit").publish();
            pubMonYellowIntakeArmVelocityDegPerSec =
                monitorNT.getDoubleTopic("IntakeArm/YEL/VelocityDegPerSec").publish();
            pubMonYellowIntakeArmAppliedVolts =
                monitorNT.getDoubleTopic("IntakeArm/YEL/AppliedVolts").publish();

            // Intake Roller
            pubMonIntakeRollerDutyCycle =
                monitorNT.getDoubleTopic("IntakeRoller/DutyCycle").publish();
            pubMonIntakeRollerTemp =
                monitorNT.getDoubleTopic("IntakeRoller/TemperatureC").publish();
            pubMonIntakeRollerCurrent =
                monitorNT.getDoubleTopic("IntakeRoller/CurrentA").publish();
            pubMonIntakeRollerCmdDutyCycle =
                monitorNT.getDoubleTopic("IntakeRoller/CmdDutyCycle").publish();

            // Shooter Blue
            pubMonBlueShooterMotorRPS =
                monitorNT.getDoubleTopic("Shooter/BLU/MotorRPS").publish();
            pubMonBlueShooterCurrentRPM =
                monitorNT.getDoubleTopic("Shooter/BLU/CurrentRPM").publish();
            pubMonBlueShooterTargetRPM =
                monitorNT.getDoubleTopic("Shooter/BLU/TargetRPM").publish();
            pubMonBlueShooterDistanceToTarget =
                monitorNT.getDoubleTopic("Shooter/BLU/DistanceToTarget").publish();
            pubMonBlueShooterTemp =
                monitorNT.getDoubleTopic("Shooter/BLU/TemperatureC").publish();
            pubMonBlueShooterCurrent =
                monitorNT.getDoubleTopic("Shooter/BLU/CurrentA").publish();

            // Shooter Yellow
            pubMonYellowShooterMotorRPS =
                monitorNT.getDoubleTopic("Shooter/YEL/MotorRPS").publish();
            pubMonYellowShooterCurrentRPM =
                monitorNT.getDoubleTopic("Shooter/YEL/CurrentRPM").publish();
            pubMonYellowShooterTargetRPM =
                monitorNT.getDoubleTopic("Shooter/YEL/TargetRPM").publish();
            pubMonYellowShooterDistanceToTarget =
                monitorNT.getDoubleTopic("Shooter/YEL/DistanceToTarget").publish();
            pubMonYellowShooterTemp =
                monitorNT.getDoubleTopic("Shooter/YEL/TemperatureC").publish();
            pubMonYellowShooterCurrent =
                monitorNT.getDoubleTopic("Shooter/YEL/CurrentA").publish();
        }
    }


    public void initDashboard() {
        SmartDashboard.putData("Auto Mode", autoChooser);
    }

    public void updateInputs() {

        double now = Timer.getFPGATimestamp();
        if (now - lastUpdate < PERIOD) return;
        lastUpdate = now;

        if (ENABLE_MANUAL) {
            updateManual();
        }

        if (ENABLE_MATCH) {
            updateMatch();
        }

        if (ENABLE_MONITOR) {
            updateMonitorAgitator();
            updateMonitorFeederBlue();
            updateMonitorFeederYellow();
            updateMonitorHopper();
            updateMonitorIntakeArm();
            updateMonitorIntakeRoller();
            updateMonitorShooterBlue();
            updateMonitorShooterYellow();
        }
    }

    private void updateMatch() {

        var bluShooterInputs = subsystems.BLUshooter().getInputs();
        pubBLUShooterRPM.set(whole(bluShooterInputs.currentRPM));
        pubBLUShooterTargetRPM.set(whole(bluShooterInputs.targetRPM));

        var yelShooterInputs = subsystems.YELshooter().getInputs();
        pubYELShooterRPM.set(whole(yelShooterInputs.currentRPM));
        pubYELShooterTargetRPM.set(whole(yelShooterInputs.targetRPM));

        var hopper = subsystems.hopper().getInputs();
        pubHopperPos.set(round(hopper.position, 1));
        pubHopperExtended.set(hopper.hopperExtended);

        var intake = subsystems.intakeArm().getInputs();
        pubIntakeBLUPos.set(round(intake.bluPositionDeg, 1));
        pubIntakeYELPos.set(round(intake.yelPositionDeg, 1));

        var roller = subsystems.intakeRoller().getInputs();
        pubIntakeRollerCurrentDC.set(round(roller.dutyCycle, 1));
        pubIntakeRollerCmdDC.set(round(roller.cmdDutyCycle, 1));

        var quest = localization.vision().getInputs();
        pubQuestNavConnected.set(quest.connected);
        pubQuestNavIsTracking.set(quest.isTracking);
        pubQuestNavBattery.set(quest.batteryPercent);
        pubQuestNavIsResetComplete.set(localization.vision().isResetComplete());

        // --- Pose Seeding via Elastic ---
        boolean seedLeft =
            matchNT.getEntry("Localization/QuestNav/SeedRight").getBoolean(false);
        if (seedLeft) {
            localization.vision().requestPoseReset(Nodes.Start.LEFT);
            matchNT.getEntry("Localization/QuestNav/SeedRight").setBoolean(false);
        }

        boolean seedCenter =
            matchNT.getEntry("Localization/QuestNav/SeedCenter").getBoolean(false);
        if (seedCenter) {
            localization.vision().requestPoseReset(Nodes.Start.CENTER);
            matchNT.getEntry("Localization/QuestNav/SeedCenter").setBoolean(false);
        }

        boolean seedRight =
            matchNT.getEntry("Localization/QuestNav/SeedLeft").getBoolean(false);
        if (seedRight) {
            localization.vision().requestPoseReset(Nodes.Start.RIGHT);
            matchNT.getEntry("Localization/QuestNav/SeedLeft").setBoolean(false);
        }

        // --- Manual Pose Seeding ---
        double poseX = matchNT.getEntry("Localization/QuestNav/ManualX").getDouble(0.0);
        double poseY = matchNT.getEntry("Localization/QuestNav/ManualY").getDouble(0.0);
        double poseTheta = matchNT.getEntry("Localization/QuestNav/ManualThetaDeg").getDouble(0.0);

        boolean resetPose =
            matchNT.getEntry("Localization/QuestNav/ManualSeedPose").getBoolean(false);

        if (resetPose) {
            localization.vision().requestPoseReset(new Pose2d(
                poseX,
                poseY,
                new Rotation2d(Math.toRadians(poseTheta))
            ));
            matchNT.getEntry("Localization/QuestNav/ManualSeedPose").setBoolean(false);
        }
    }

    private void updateManual() {
        boolean agitator = subAgitatorRun.get();
        boolean bluFeeder = subBLUFeederRun.get();
        boolean yelFeeder = subYELFeederRun.get();
        boolean hopperRun = subHopperRun.get();
        boolean intakeArm = subIntakeArmRun.get();
        boolean intakeRoller = subIntakeRollerRun.get();
        boolean bluShooterRun = subBLUShooterRun.get();
        boolean yelShooterRun = subYELShooterRun.get();

        if (agitator && !prevAgitator)
            CommandScheduler.getInstance().schedule(
                AgitatorCommands.manual(
                    subsystems.agitator(),
                    () -> round(subAgitatorDuty.get(), 2)));

        if (bluFeeder && !prevBLUFeeder)
            CommandScheduler.getInstance().schedule(
                FeederCommands.manual(
                    subsystems.BLUfeeder(),
                    () -> whole(subBLUFeederRPM.get())));

        if (yelFeeder && !prevYELFeeder)
            CommandScheduler.getInstance().schedule(
                FeederCommands.manual(
                    subsystems.YELfeeder(),
                    () -> whole(subYELFeederRPM.get())));

        if (hopperRun && !prevHopper)
            CommandScheduler.getInstance().schedule(
                HopperCommands.manualPosition(
                    subsystems.hopper(),
                    () -> round(subHopperAngle.get(), 1)));

        if (intakeArm && !prevIntakeArm)
            CommandScheduler.getInstance().schedule(
                IntakeArmCommands.manualAngle(
                    subsystems.intakeArm(),
                    () -> whole(subIntakeArmAngle.get())));

        if (intakeRoller && !prevIntakeRoller)
            CommandScheduler.getInstance().schedule(
                IntakeRollerCommands.manual(
                    subsystems.intakeRoller(),
                    () -> round(subIntakeRollerDuty.get(), 2)));

        if (bluShooterRun && !prevBLUShooter)
            CommandScheduler.getInstance().schedule(
                ShooterCommands.manual(
                    subsystems.BLUshooter(),
                    () -> whole(subBLUShooterRPM.get())));

        if (yelShooterRun && !prevYELShooter)
            CommandScheduler.getInstance().schedule(
                ShooterCommands.manual(
                    subsystems.YELshooter(),
                    () -> whole(subYELShooterRPM.get())));

        if (!agitator && prevAgitator) subsystems.agitator().getCurrentCommand().cancel();
        if (!bluFeeder && prevBLUFeeder) subsystems.BLUfeeder().getCurrentCommand().cancel();
        if (!yelFeeder && prevYELFeeder) subsystems.YELfeeder().getCurrentCommand().cancel();
        if (!hopperRun && prevHopper) subsystems.hopper().getCurrentCommand().cancel();
        if (!intakeArm && prevIntakeArm) subsystems.intakeArm().getCurrentCommand().cancel();
        if (!intakeRoller && prevIntakeRoller) subsystems.intakeRoller().getCurrentCommand().cancel();
        if (!bluShooterRun && prevBLUShooter) subsystems.BLUshooter().getCurrentCommand().cancel();
        if (!yelShooterRun && prevYELShooter) subsystems.YELshooter().getCurrentCommand().cancel();

        prevAgitator = agitator;
        prevBLUFeeder = bluFeeder;
        prevYELFeeder = yelFeeder;
        prevHopper = hopperRun;
        prevIntakeArm = intakeArm;
        prevIntakeRoller = intakeRoller;
        prevBLUShooter = bluShooterRun;
        prevYELShooter = yelShooterRun;
    }


    private void updateMonitorAgitator() {
        var inputs = subsystems.agitator().getInputs();

        pubMonAgitatorDuty.set(round(inputs.dutyCycle, 2));
        pubMonAgitatorTemp.set(round(inputs.temperatureC, 1));
        pubMonAgitatorCurrent.set(round(inputs.currentA, 1));
        pubMonAgitatorFeeding.set(inputs.feedingEnabled);
    }

    private void updateMonitorFeederBlue() {
        var inputs = subsystems.BLUfeeder().getInputs();

        pubMonBlueFeederRPM.set(whole(inputs.velocityRPM));
        pubMonBlueFeederRPS.set(round(inputs.motorRPS, 1));
        pubMonBlueFeederTemp.set(round(inputs.temperatureC, 1));
        pubMonBlueFeederCurrent.set(round(inputs.currentA, 1));
    }

    private void updateMonitorFeederYellow() {
        var inputs = subsystems.YELfeeder().getInputs();

        pubMonYellowFeederRPM.set(whole(inputs.velocityRPM));
        pubMonYellowFeederRPS.set(round(inputs.motorRPS, 1));
        pubMonYellowFeederTemp.set(round(inputs.temperatureC, 1));
        pubMonYellowFeederCurrent.set(round(inputs.currentA, 1));
    }

    private void updateMonitorHopper() {
        var inputs = subsystems.hopper().getInputs();

        pubMonHopperPosition.set(round(inputs.position, 1));
        pubMonHopperMagSensor.set(inputs.magSensor);
        pubMonHopperTemp.set(round(inputs.temperatureC, 1));
        pubMonHopperCurrent.set(round(inputs.currentA, 1));
        pubMonHopperRetracted.set(inputs.hopperRetracted);
        pubMonHopperExtended.set(inputs.hopperExtended);
        pubMonHopperReverseLimit.set(inputs.reverseLimit);
    }

    private void updateMonitorIntakeArm() {
        var inputs = subsystems.intakeArm().getInputs();

        pubMonBlueIntakeArmPosition.set(whole(inputs.bluPositionDeg));
        pubMonBlueIntakeArmReverseLimit.set(inputs.bluReverseLimit);
        pubMonBlueIntakeArmMotorPosition.set(round(inputs.bluMotorRot, 1));
        pubMonBlueIntakeArmTemp.set(round(inputs.bluTempC, 1));
        pubMonBlueIntakeArmCurrent.set(round(inputs.bluCurrentA, 1));
        pubMonBlueIntakeArmAppliedVolts.set(round(inputs.bluAppliedVolts, 2));
        pubMonBlueIntakeArmVelocityDegPerSec.set(round(inputs.bluVelocityDegPerSec, 2));
        
        pubMonYellowIntakeArmPosition.set(whole(inputs.yelPositionDeg));
        pubMonYellowIntakeArmReverseLimit.set(inputs.yelReverseLimit);
        pubMonYellowIntakeArmMotorPosition.set(round(inputs.yelMotorRot, 1));
        pubMonYellowIntakeArmTemp.set(round(inputs.yelTempC, 1));
        pubMonYellowIntakeArmCurrent.set(round(inputs.yelCurrentA, 1));
        pubMonYellowIntakeArmAppliedVolts.set(round(inputs.yelAppliedVolts, 2));
        pubMonYellowIntakeArmVelocityDegPerSec.set(round(inputs.yelVelocityDegPerSec, 2));
    }

    private void updateMonitorIntakeRoller() {
        var inputs = subsystems.intakeRoller().getInputs();

        pubMonIntakeRollerDutyCycle.set(round(inputs.dutyCycle, 2));
        pubMonIntakeRollerTemp.set(round(inputs.temperatureC, 1));
        pubMonIntakeRollerCurrent.set(round(inputs.currentA, 1));
        pubMonIntakeRollerCmdDutyCycle.set(round(inputs.cmdDutyCycle, 2));
    }

    private void updateMonitorShooterBlue() {
        var inputs = subsystems.BLUshooter().getInputs();

        pubMonBlueShooterMotorRPS.set(round(inputs.motorRPS, 1));
        pubMonBlueShooterCurrentRPM.set(whole(inputs.currentRPM));
        pubMonBlueShooterTargetRPM.set(whole(inputs.targetRPM));
        pubMonBlueShooterDistanceToTarget.set(round(inputs.distanceToTarget, 2));
        pubMonBlueShooterTemp.set(round(inputs.temperatureC, 1));
        pubMonBlueShooterCurrent.set(round(inputs.currentA, 1));
    }

    private void updateMonitorShooterYellow() {
        var inputs = subsystems.YELshooter().getInputs();

        pubMonYellowShooterMotorRPS.set(round(inputs.motorRPS, 1));
        pubMonYellowShooterCurrentRPM.set(whole(inputs.currentRPM));
        pubMonYellowShooterTargetRPM.set(whole(inputs.targetRPM));
        pubMonYellowShooterDistanceToTarget.set(round(inputs.distanceToTarget, 2));
        pubMonYellowShooterTemp.set(round(inputs.temperatureC, 1));
        pubMonYellowShooterCurrent.set(round(inputs.currentA, 1));
    }
}
