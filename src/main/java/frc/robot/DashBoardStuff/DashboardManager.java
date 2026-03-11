//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.robot.DashBoardStuff;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    private final BooleanPublisher pubHasGoodVision =
        nt.getBooleanTopic("Localization/PhotonVision/HasTargets").publish();

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

        pubHasGoodVision.set(localization.vision() != null);

      

       
    }
}
