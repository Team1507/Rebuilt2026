//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.robot;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.atomic.*;
import frc.robot.framework.*;

public class DashboardManager {

    private final SubsystemsRecord subsystems;
    private final LocalizationRecord localization;
    private final SendableChooser<?> autoChooser;

    /* ---------------- NT Root ---------------- */
    private final NetworkTable nt =
        NetworkTableInstance.getDefault().getTable("SmartDashboard");

    /* ---------------- Throttle ---------------- */
    private double lastUpdate = 0.0;
    private static final double PERIOD = 0.20;   // 5 Hz

    /* ---------------- Manual Subscribers ---------------- */
    private final DoubleSubscriber subAgitatorDuty =
        nt.getDoubleTopic("Manual Mode/Agitator/Target DC").subscribe(0.0);
    private final DoubleSubscriber subClimberPos =
        nt.getDoubleTopic("Manual Mode/Climber/Target Position").subscribe(0.0);
    private final DoubleSubscriber subBLUFeederRPM =
        nt.getDoubleTopic("Manual Mode/Feeder/BLU/Target RPM").subscribe(0.0);
    private final DoubleSubscriber subYELFeederRPM =
        nt.getDoubleTopic("Manual Mode/Feeder/YEL/Target RPM").subscribe(0.0);
    private final DoubleSubscriber subHopperAngle =
        nt.getDoubleTopic("Manual Mode/Hopper/Target Angle").subscribe(0.0);
    private final DoubleSubscriber subIntakeArmAngle =
        nt.getDoubleTopic("Manual Mode/Intake/Arm/Target Angle").subscribe(0.0);
    private final DoubleSubscriber subIntakeRollerDuty =
        nt.getDoubleTopic("Manual Mode/Intake/Roller/Target DC").subscribe(0.0);
    private final DoubleSubscriber subBLUShooterRPM =
        nt.getDoubleTopic("Manual Mode/Shooter/BLU/Target RPM").subscribe(0.0);
    private final DoubleSubscriber subYELShooterRPM =
        nt.getDoubleTopic("Manual Mode/Shooter/YEL/Target RPM").subscribe(0.0);
    private final DoubleSubscriber subShooterIdleRPM =
        nt.getDoubleTopic("Shooter/Shooter Idle RPM").subscribe(0.0);

    /* ---------------- RUN Button Subscribers ---------------- */
    private final BooleanSubscriber subAgitatorRun =
        nt.getBooleanTopic("Manual Mode/Agitator/Run").subscribe(false);
    private final BooleanSubscriber subClimberRun =
        nt.getBooleanTopic("Manual Mode/Climber/Run").subscribe(false);
    private final BooleanSubscriber subBLUFeederRun =
        nt.getBooleanTopic("Manual Mode/Feeder/BLU/Run").subscribe(false);
    private final BooleanSubscriber subYELFeederRun =
        nt.getBooleanTopic("Manual Mode/Feeder/YEL/Run").subscribe(false);
    private final BooleanSubscriber subHopperRun =
        nt.getBooleanTopic("Manual Mode/Hopper/Run").subscribe(false);
    private final BooleanSubscriber subIntakeArmRun =
        nt.getBooleanTopic("Manual Mode/Intake/Arm/Run").subscribe(false);
    private final BooleanSubscriber subIntakeRollerRun =
        nt.getBooleanTopic("Manual Mode/Intake/Roller/Run").subscribe(false);
    private final BooleanSubscriber subBLUShooterRun =
        nt.getBooleanTopic("Manual Mode/Shooter/BLU/Run").subscribe(false);
    private final BooleanSubscriber subYELShooterRun =
        nt.getBooleanTopic("Manual Mode/Shooter/YEL/Run").subscribe(false);

    /* ---------------- Publishers ---------------- */
    private final DoublePublisher pubClimberPos =
        nt.getDoubleTopic("Climber/CurrentPosition").publish();

    private final DoublePublisher pubHopperPos =
        nt.getDoubleTopic("Hopper/CurrentPosition").publish();
    private final BooleanPublisher pubHopperExtended =
        nt.getBooleanTopic("Hopper/IsExtended").publish();

    private final DoublePublisher pubIntakeBLUPos =
        nt.getDoubleTopic("Intake/Arm/BLU/Current Position").publish();
    private final DoublePublisher pubIntakeYELPos =
        nt.getDoubleTopic("Intake/Arm/YEL/Current Position").publish();
    private final BooleanPublisher pubIntakeBLURev =
        nt.getBooleanTopic("Intake/Arm/BLU/ReverseLimit").publish();
    private final BooleanPublisher pubIntakeYELRev =
        nt.getBooleanTopic("Intake/Arm/YEL/ReverseLimit").publish();

    private final BooleanPublisher pubBLUShooterSensor =
        nt.getBooleanTopic("Shooter/BLU/Sensor").publish();
    private final DoublePublisher pubBLUShooterDist =
        nt.getDoubleTopic("Shooter/BLU/Distance To Target").publish();
    private final DoublePublisher pubBLUShooterRPM =
        nt.getDoubleTopic("Shooter/BLU/Current RPM").publish();
    private final DoublePublisher pubBLUShooterTargetRPM =
        nt.getDoubleTopic("Shooter/BLU/Target RPM").publish();

    private final BooleanPublisher pubYELShooterSensor =
        nt.getBooleanTopic("Shooter/YEL/Sensor").publish();
    private final DoublePublisher pubYELShooterDist =
        nt.getDoubleTopic("Shooter/YEL/Distance To Target").publish();
    private final DoublePublisher pubYELShooterRPM =
        nt.getDoubleTopic("Shooter/YEL/Current RPM").publish();
        private final DoublePublisher pubYELShooterTargetRPM =
        nt.getDoubleTopic("Shooter/YEL/Target RPM").publish();

    /* ---------------- Localization ---------------- */
    private final BooleanPublisher pubSeeded =
        nt.getBooleanTopic("Localization/PoseSeeded").publish();
    private final BooleanPublisher pubLocalizationResetSeed =
        nt.getBooleanTopic("Localization/ResetPoseSeed").publish();
    private final DoublePublisher pubQuestBattery =
        nt.getDoubleTopic("Localization/QuestNav/BatteryPercent").publish();
    private final BooleanPublisher pubQuestTracking =
        nt.getBooleanTopic("Localization/QuestNav/Is Tracking").publish();
    private final BooleanPublisher pubHasGoodVision =
        nt.getBooleanTopic("Localization/PhotonVision/Has Good Vision").publish();

    /* ---------------- PV Debug ---------------- */
    private final BooleanPublisher pubPVGood =
        nt.getBooleanTopic("PhotonVision/Fused/HasGoodVision").publish();
    private final BooleanPublisher pubPVSeeded =
        nt.getBooleanTopic("PhotonVision/Fused/Seeded").publish();
    private final DoublePublisher pubPVX =
        nt.getDoubleTopic("PhotonVision/Fused/Pose/X").publish();
    private final DoublePublisher pubPVY =
        nt.getDoubleTopic("PhotonVision/Fused/Pose/Y").publish();
    private final DoublePublisher pubPVHeading =
        nt.getDoubleTopic("PhotonVision/Fused/Pose/HeadingDeg").publish();
    private final DoublePublisher pubPVXYStd =
        nt.getDoubleTopic("PhotonVision/Fused/StdDev/XY").publish();
    private final DoublePublisher pubPVAngStd =
        nt.getDoubleTopic("PhotonVision/Fused/StdDev/Ang").publish();

    /* ---------------- Rising-edge tracking ---------------- */
    private boolean prevAgitator, prevClimber, prevBLUFeeder, prevYELFeeder;
    private boolean prevHopper, prevIntakeArm, prevIntakeRoller;
    private boolean prevBLUShooter, prevYELShooter;

    public DashboardManager(
        SubsystemsRecord subsystems,
        LocalizationRecord localization,
        SendableChooser<?> autoChooser
    ) {
        this.subsystems = subsystems;
        this.localization = localization;
        this.autoChooser = autoChooser;
    }

    public void initDashboard() {
        SmartDashboard.putData("Auto Mode", autoChooser);
        pubLocalizationResetSeed.set(false);
    }

    public void updateInputs() {

        double now = Timer.getFPGATimestamp();
        if (now - lastUpdate < PERIOD) return;
        lastUpdate = now;

        /* ---------------- Subsystem Inputs ---------------- */
        var climber = subsystems.climber().getInputs();
        pubClimberPos.set(climber.position);

        var hopper = subsystems.hopper().getInputs();
        pubHopperPos.set(hopper.position);
        pubHopperExtended.set(hopper.hopperRetracted);

        var intake = subsystems.intakeArm().getInputs();
        pubIntakeBLUPos.set(intake.bluPositionDeg);
        pubIntakeYELPos.set(intake.yelPositionDeg);
        pubIntakeBLURev.set(intake.bluReverseLimit);
        pubIntakeYELRev.set(intake.yelReverseLimit);

        var bluShooter = subsystems.BLUshooter().getInputs();
        pubBLUShooterRPM.set(subsystems.BLUshooter().getShooterRPM());
        pubBLUShooterSensor.set(bluShooter.ballFired);
        pubBLUShooterDist.set(bluShooter.distanceToTarget);
        pubBLUShooterTargetRPM.set(subsystems.BLUshooter().getTargetRPM());

        var yelShooter = subsystems.YELshooter().getInputs();
        pubYELShooterRPM.set(subsystems.YELshooter().getShooterRPM());
        pubYELShooterSensor.set(yelShooter.ballFired);
        pubYELShooterDist.set(yelShooter.distanceToTarget);
        pubYELShooterTargetRPM.set(subsystems.YELshooter().getTargetRPM());

        /* ---------------- Localization ---------------- */
        pubSeeded.set(localization.localizationManager().isStartupSeeded());

        boolean resetSeed =
            nt.getEntry("Localization/ResetPoseSeed").getBoolean(false);

        if (resetSeed) {
            localization.localizationManager().resetVisionSeed();
            nt.getEntry("Localization/ResetPoseSeed").setBoolean(false);
        }

        pubQuestBattery.set(localization.questNav().getBatteryPercent().orElse(-1));
        pubQuestTracking.set(localization.questNav().isTracking());
        pubHasGoodVision.set(localization.pvManager().hasGoodVision());

        /* ---------------- PhotonVision Debug ---------------- */
        var pv = localization.pvManager().getDebugInfo();

        pubPVGood.set(pv.fusedValid);
        pubPVSeeded.set(pv.seeded);

        if (pv.fusedValid) {
            pubPVX.set(pv.fusedPose.getX());
            pubPVY.set(pv.fusedPose.getY());
            pubPVHeading.set(pv.fusedPose.getRotation().getDegrees());
            pubPVXYStd.set(pv.fusedXyStd);
            pubPVAngStd.set(pv.fusedAngStd);
        }

        /* ---------------- Manual RUN Buttons ---------------- */
        boolean agitator = subAgitatorRun.get();
        boolean climberRun = subClimberRun.get();
        boolean bluFeeder = subBLUFeederRun.get();
        boolean yelFeeder = subYELFeederRun.get();
        boolean hopperRun = subHopperRun.get();
        boolean intakeArm = subIntakeArmRun.get();
        boolean intakeRoller = subIntakeRollerRun.get();
        boolean bluShooterRun = subBLUShooterRun.get();
        boolean yelShooterRun = subYELShooterRun.get();

        /* Rising edges */
        if (agitator && !prevAgitator)
            AgitatorCommands.manual(subsystems.agitator(), () -> subAgitatorDuty.get()).schedule();
        if (climberRun && !prevClimber)
            ClimberCommands.manual(subsystems.climber(), () -> subClimberPos.get()).schedule();
        if (bluFeeder && !prevBLUFeeder)
            FeederCommands.manual(subsystems.BLUfeeder(), () -> subBLUFeederRPM.get()).schedule();
        if (yelFeeder && !prevYELFeeder)
            FeederCommands.manual(subsystems.YELfeeder(), () -> subYELFeederRPM.get()).schedule();
        if (hopperRun && !prevHopper)
            HopperCommands.manualPosition(subsystems.hopper(), () -> subHopperAngle.get()).schedule();
        if (intakeArm && !prevIntakeArm)
            IntakeArmCommands.manualAngle(subsystems.intakeArm(), () -> subIntakeArmAngle.get()).schedule();
        if (intakeRoller && !prevIntakeRoller)
            IntakeRollerCommands.manual(subsystems.intakeRoller(), () -> subIntakeRollerDuty.get()).schedule();
        if (bluShooterRun && !prevBLUShooter)
            ShooterCommands.manual(subsystems.BLUshooter(), () -> subBLUShooterRPM.get()).schedule();
        if (yelShooterRun && !prevYELShooter)
            ShooterCommands.manual(subsystems.YELshooter(), () -> subYELShooterRPM.get()).schedule();

        /* Falling edges */
        if (!agitator && prevAgitator) subsystems.agitator().getCurrentCommand().cancel();
        if (!climberRun && prevClimber) subsystems.climber().getCurrentCommand().cancel();
        if (!bluFeeder && prevBLUFeeder) subsystems.BLUfeeder().getCurrentCommand().cancel();
        if (!yelFeeder && prevYELFeeder) subsystems.YELfeeder().getCurrentCommand().cancel();
        if (!hopperRun && prevHopper) subsystems.hopper().getCurrentCommand().cancel();
        if (!intakeArm && prevIntakeArm) subsystems.intakeArm().getCurrentCommand().cancel();
        if (!intakeRoller && prevIntakeRoller) subsystems.intakeRoller().getCurrentCommand().cancel();
        if (!bluShooterRun && prevBLUShooter) subsystems.BLUshooter().getCurrentCommand().cancel();
        if (!yelShooterRun && prevYELShooter) subsystems.YELshooter().getCurrentCommand().cancel();

        /* Update previous */
        prevAgitator = agitator;
        prevClimber = climberRun;
        prevBLUFeeder = bluFeeder;
        prevYELFeeder = yelFeeder;
        prevHopper = hopperRun;
        prevIntakeArm = intakeArm;
        prevIntakeRoller = intakeRoller;
        prevBLUShooter = bluShooterRun;
        prevYELShooter = yelShooterRun;
    }

    public double getShooterIdleRPM() {
        return subShooterIdleRPM.get();
    }
}
