// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.framework.LocalizationRecord;
import frc.robot.framework.SubsystemsRecord;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.atomic.*;
import frc.robot.framework.*;

/** Add your docs here. */
public class DashboardManagerMatch 
{
    private final SubsystemsRecord subsystems;
    private final LocalizationRecord localization;
    private final SendableChooser<?> autoChooser;

    private final NetworkTable nt =
        NetworkTableInstance.getDefault().getTable("Match");

    private double lastUpdate = 0.0;
    private static final double PERIOD = 0.20;   // 5 Hz

    //stuff

    //blue shooter
    private final DoublePublisher pubBLUShooterRPM =
        nt.getDoubleTopic("Shooter/BLU/Current RPM").publish();
    private final DoublePublisher pubBLUShooterTargetRPM =
        nt.getDoubleTopic("Shooter/BLU/Target RPM").publish();

    //yellow shooter
    private final DoublePublisher pubYELShooterRPM =
        nt.getDoubleTopic("Shooter/YEL/Current RPM").publish();
        private final DoublePublisher pubYELShooterTargetRPM =
        nt.getDoubleTopic("Shooter/YEL/Target RPM").publish();

    //hopper
    private final DoublePublisher pubHopperPos =
            nt.getDoubleTopic("Hopper/CurrentPosition").publish();
    private final BooleanPublisher pubHopperExtended =
        nt.getBooleanTopic("Hopper/IsExtended").publish();

    //intake
    private final DoublePublisher pubIntakeBLUPos =
        nt.getDoubleTopic("Intake/Arm/BLU/Current Position").publish();
    private final DoublePublisher pubIntakeYELPos =
        nt.getDoubleTopic("Intake/Arm/YEL/Current Position").publish();

    //visoion
    private final BooleanPublisher pubSeeded =
        nt.getBooleanTopic("Localization/PoseSeeded").publish();
    private final BooleanPublisher pubLocalizationResetSeed =
        nt.getBooleanTopic("Localization/ResetPoseSeed").publish();
    private final DoublePublisher pubQuestBattery =
        nt.getDoubleTopic("Localization/QuestNav/BatteryPercent").publish();
    private final BooleanPublisher pubHasGoodVision =
        nt.getBooleanTopic("Localization/PhotonVision/Has Good Vision").publish();

    private final DoublePublisher pubSetPoseX =
        nt.getDoubleTopic("Set Pose/X").publish();

    private final DoublePublisher pubSetPoseY =
        nt.getDoubleTopic("Set Pose/Y").publish();

    private final DoublePublisher pubSetPoseTheta =
        nt.getDoubleTopic("Set Pose/Theta").publish();

    private final BooleanPublisher pubSetRobotPose =
        nt.getBooleanTopic("Set Pose/Reset Pose").publish();

    public DashboardManagerMatch(
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

        //blue shoot
        var bluShooter = subsystems.BLUshooter().getInputs();
        pubBLUShooterRPM.set(subsystems.BLUshooter().getShooterRPM());
        pubBLUShooterTargetRPM.set(subsystems.BLUshooter().getTargetRPM());

        //yellow shooter
        var yelShooter = subsystems.YELshooter().getInputs();
        pubYELShooterRPM.set(subsystems.YELshooter().getShooterRPM());
        pubYELShooterTargetRPM.set(subsystems.YELshooter().getTargetRPM());

        //hopper
        var hopper = subsystems.hopper().getInputs();
        pubHopperPos.set(hopper.position);
        pubHopperExtended.set(hopper.hopperRetracted);


        //intake
        var intake = subsystems.intakeArm().getInputs();
        pubIntakeBLUPos.set(intake.bluPositionDeg);
        pubIntakeYELPos.set(intake.yelPositionDeg);

        //quest
        pubSeeded.set(localization.localizationManager().isStartupSeeded());

        boolean resetSeed =
            nt.getEntry("Localization/ResetPoseSeed").getBoolean(false);

        if (resetSeed) {
            localization.localizationManager().resetVisionSeed();
            nt.getEntry("Localization/ResetPoseSeed").setBoolean(false);
        }

        pubQuestBattery.set(localization.questNav().getBatteryPercent().orElse(-1));
        pubHasGoodVision.set(localization.pvManager().hasGoodVision());

        double poseX = nt.getEntry("Set Pose/X").getDouble(0.0);
        double poseY = nt.getEntry("Set Pose/Y").getDouble(0.0);
        double poseTheta = nt.getEntry("Set Pose/Theta").getDouble(0.0);

        boolean resetPose =
            nt.getEntry("Set Pose/Reset Pose").getBoolean(false);

        if (resetPose) {
            localization.localizationManager().resetQuestPose(new Pose2d(
                poseX,
                poseY,
                new Rotation2d(poseTheta)
            ));
            nt.getEntry("Set Pose/Reset Pose").setBoolean(false);
        }
    }
}
