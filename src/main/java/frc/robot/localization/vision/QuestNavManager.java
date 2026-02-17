//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.robot.localization.vision;

import java.util.OptionalDouble;
import java.util.OptionalInt;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.lib.logging.Telemetry;
import frc.robot.Constants.kQuest;
import frc.robot.generated.questnav.PoseFrame;
import frc.robot.generated.questnav.QuestNav;

/**
 * QuestNavManager:
 *  - Reads QuestNav pose frames
 *  - Applies ROBOT_TO_QUEST transform
 *  - Produces corrected robot pose
 *  - Logs tracking, latency, battery, timestamps
 *
 * This class does NOT:
 *  - Fuse with odometry
 *  - Fuse with PhotonVision
 *  - Seed the drivetrain
 *  - Seed QuestNav externally
 *
 * Those responsibilities belong to LocalizationManager.
 */
public class QuestNavManager extends SubsystemBase {

    private final QuestNav questNav = new QuestNav();
    private final Telemetry telemetry;

    // Latest processed output
    private Pose2d latestPose = new Pose2d();
    private double latestTimestamp = 0.0;
    private boolean latestTracking = false;

    // Throttle to 20 Hz
    private double lastProcessTime = 0.0;
    private static final double PROCESS_PERIOD = 0.05;

    public QuestNavManager(Telemetry telemetry) {
        this.telemetry = telemetry;

        telemetry.registerVisionPoseSource("QuestNav-Raw");
        telemetry.registerVisionPoseSource("QuestNav-Corrected");
    }

    @Override
    public void periodic() {

        questNav.commandPeriodic();

        double now = Timer.getFPGATimestamp();
        if (now - lastProcessTime < PROCESS_PERIOD) return;
        lastProcessTime = now;

        telemetry.logQuestNavConnected(questNav.isConnected());

        PoseFrame[] frames = questNav.getAllUnreadPoseFrames();
        if (frames.length == 0) {
            latestTracking = false;
            telemetry.logQuestNavTracking(false);
            return;
        }

        // Only process the latest frame
        PoseFrame frame = frames[frames.length - 1];
        latestTracking = frame.isTracking();
        telemetry.logQuestNavTracking(latestTracking);

        if (!latestTracking) {
            return;
        }

        Pose3d questPose = frame.questPose3d();
        double frameTimestamp = frame.dataTimestamp();

        telemetry.logQuestNavRawPose(questPose);
        telemetry.logQuestNavFrameTimestamp(frameTimestamp);

        // Transform QuestNav pose into robot frame
        Pose3d robotPose3d = questPose.transformBy(kQuest.ROBOT_TO_QUEST.inverse());

        // QuestNav heading is already drift‑corrected
        Rotation2d heading = new Rotation2d(robotPose3d.getRotation().getZ());

        latestPose = new Pose2d(
            robotPose3d.getX(),
            robotPose3d.getY(),
            heading
        );

        latestTimestamp = frameTimestamp;

        telemetry.logQuestNavCorrectedPose(latestPose);

        double latencyMs = (now - frameTimestamp) * 1000.0;
        telemetry.logQuestNavLatency(latencyMs);

        questNav.getBatteryPercent().ifPresent(telemetry::logQuestNavBattery);
        questNav.getAppTimestamp().ifPresent(telemetry::logQuestNavAppTimestamp);
    }

    // ============================
    // Public API for LocalizationManager
    // ============================

    public boolean isConnected() {
        return questNav.isConnected();
    }

    public boolean isTracking() {
        return latestTracking;
    }

    public Pose2d getPose2d() {
        return latestPose;
    }

    public double getTimestamp() {
        return latestTimestamp;
    }

    public OptionalInt getBatteryPercent() {
        return questNav.getBatteryPercent();
    }

    public OptionalDouble getAppTimestamp() {
        return questNav.getAppTimestamp();
    }

    /**
     * Seeds QuestNav pose
     */
    public void seedPose(Pose3d pose) {
        questNav.setPose(pose);
    }
}
