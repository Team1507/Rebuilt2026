package frc.robot.subsystems;

import java.util.OptionalDouble;
import java.util.OptionalInt;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.quest.PoseFrame;
import frc.robot.subsystems.quest.QuestNav;
import frc.robot.Constants.kQuest;

public class QuestNavSubsystem extends SubsystemBase {

    private final CommandSwerveDrivetrain drivetrain;
    private final QuestNav questNav = new QuestNav();

    // Throttle QuestNav processing to 20 Hz
    private double lastProcessTime = 0.0;
    private static final double PROCESS_PERIOD = 0.05; // 50 ms

    public QuestNavSubsystem(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    @Override
    public void periodic() {

        questNav.commandPeriodic();

        double now = Timer.getFPGATimestamp();
        if (now - lastProcessTime < PROCESS_PERIOD) {
            return;
        }
        lastProcessTime = now;

        PoseFrame[] frames = questNav.getAllUnreadPoseFrames();
        if (frames.length == 0) return;

        // Only process the latest frame
        PoseFrame frame = frames[frames.length - 1];

        if (!frame.isTracking()) return;

        Pose3d questPose = frame.questPose3d();
        double timestamp = frame.dataTimestamp();

        Pose3d robotPose = questPose.transformBy(kQuest.ROBOT_TO_QUEST.inverse());

        Rotation2d pigeonHeading = drivetrain.getHeading();

        Pose2d correctedPose = new Pose2d(
            robotPose.getX(),
            robotPose.getY(),
            pigeonHeading
        );

        drivetrain.addVisionMeasurement(correctedPose, timestamp, kQuest.STD_DEVS);
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
