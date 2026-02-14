package frc.robot.subsystems;

import java.util.OptionalDouble;
import java.util.OptionalInt;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.quest.PoseFrame;
import frc.robot.subsystems.quest.QuestNav;
import frc.robot.Constants.kQuest;

public class QuestNavSubsystem extends SubsystemBase {

    private final CommandSwerveDrivetrain drivetrain;
    private final QuestNav questNav = new QuestNav();

    public QuestNavSubsystem(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    @Override
    public void periodic() {
        QuestNav questNav = new QuestNav();

        questNav.commandPeriodic();

        PoseFrame[] questFrames = questNav.getAllUnreadPoseFrames();

        // Loop over the pose data frames and send them to the pose estimator
        for (PoseFrame questFrame : questFrames) {
            // Make sure the Quest was tracking the pose for this frame
            if (questFrame.isTracking()) {
                // Get the pose of the Quest
                Pose3d questPose = questFrame.questPose3d();
                // Get timestamp for when the data was sent
                double timestamp = questFrame.dataTimestamp();

                // Transform by the mount pose to get your robot pose
                Pose3d robotPose = questPose.transformBy(kQuest.ROBOT_TO_QUEST.inverse());

                // You can put some sort of filtering here if you would like!

                Rotation2d pigeonHeading = drivetrain.getHeading(); // however you access it

                Pose2d correctedPose = new Pose2d(
                    robotPose.getX(),
                    robotPose.getY(),
                    pigeonHeading
                );

                // Add the measurement to our estimator
                drivetrain.addVisionMeasurement(correctedPose, timestamp, kQuest.STD_DEVS);
            }
        }
    }

    public boolean isConnected() {
        return questNav.isConnected();
    }

    public OptionalInt getBatteryPercent() {
        return questNav.getBatteryPercent();
    }

    public OptionalDouble getAppTimestamp() {
        return questNav.getAppTimestamp();
    }

    public void setQuestNavPose(Pose3d pose) {
        questNav.setPose(pose);
    }
}
