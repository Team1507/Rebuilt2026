//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.robot.localization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

import frc.robot.subsystems.QuestNavSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.lib.io.questnav.QuestNavInputs;
import frc.robot.Constants.kQuest;
import frc.robot.framework.base.Subsystems1507;

/**
 * QuestNavController
 *
 * This class owns all coordination between QuestNav and the drivetrain pose
 * estimator. It provides a unified, timing‑safe way to:
 *
 *  • Feed QuestNav vision into the drivetrain
 *  • Reset both QuestNav and drivetrain poses
 *  • Ensure resets occur only when QuestNav is ready (fresh tracking frame)
 *  • Avoid stale reset commands that QuestNav would reject
 *
 * The controller acts as a behavioral layer: autos, driver buttons, and
 * subsystems request pose resets through {@link #requestPoseReset(Pose2d)},
 * and the controller performs the reset at the earliest safe moment.
 *
 * This architecture ensures deterministic behavior without requiring callers
 * to understand QuestNav timing, tracking state, or command freshness.
 */
public class QuestNavController extends Subsystems1507 {

    private final SwerveSubsystem swerve;
    private final QuestNavSubsystem quest;

    /** Throttled periodic loop (20 Hz) */
    private double lastProcess = 0.0;
    private static final double PERIOD = 0.05;

    /** A requested pose reset waiting for QuestNav to be ready */
    private Pose2d pendingReset = null;

    /** True once QuestNav has produced at least one valid tracking frame */
    private boolean hasSeenFreshFrame = false;

    /** True after the last commanded reset completed */
    private boolean lastResetComplete = false;

    public QuestNavController(SwerveSubsystem swerve, QuestNavSubsystem quest) {
        this.swerve = swerve;
        this.quest = quest;
    }

    @Override
    public void periodic() {

        // Throttle to 20 Hz to match QuestNav's command processing expectations
        double now = Timer.getFPGATimestamp();
        if (now - lastProcess < PERIOD) return;
        lastProcess = now;

        boolean tracking = quest.isTracking();
        Pose2d questPose = quest.getPose();
        double questTs = quest.getTimestamp();

        // Track whether QuestNav has produced at least one fresh frame
        if (tracking) {
            hasSeenFreshFrame = true;
        }

        // ------------------------------------------------------------
        // 1. Perform pending pose reset when QuestNav is ready
        // ------------------------------------------------------------
        //
        // Requirements:
        //  • A reset was requested
        //  • QuestNav is tracking
        //  • QuestNav has produced at least one fresh frame
        //
        // This guarantees the reset command will NOT be stale and will be
        // accepted by QuestNav's internal command processor.
        //
        if (pendingReset != null) {
            if (tracking && hasSeenFreshFrame) {
                // Normal case: QuestNav is ready
                performReset(pendingReset);
            } else if (!quest.isConnected()) {
                // Fallback case: QuestNav is dead/unplugged
                performSwerveOnlyReset(pendingReset);
            }
        }

        // ------------------------------------------------------------
        // 2. Feed QuestNav vision into the drivetrain pose estimator
        // ------------------------------------------------------------
        //
        // QuestNav is the ONLY vision source. We always feed it when tracking.
        //
        if (tracking) {
            swerve.addVisionMeasurement(
                questPose,
                questTs,
                kQuest.STD_DEVS
            );
        }
    }

    /**
     * Request a pose reset.
     *
     * This does NOT immediately reset the robot. Instead, the reset is queued
     * and will be performed at the earliest safe moment:
     *
     *  • QuestNav must be tracking
     *  • QuestNav must have produced at least one fresh frame
     *
     * This prevents stale reset commands and ensures deterministic behavior
     * during autos and driver‑initiated resets.
     *
     * @param pose The desired field pose for both QuestNav and the drivetrain.
     */
    public void requestPoseReset(Pose2d pose) {
        pendingReset = pose;
        lastResetComplete = false;
    }

    /**
     * Immediately performs a pose reset on both QuestNav and the drivetrain.
     *
     * This should ONLY be called internally by the controller once it has
     * verified that QuestNav is ready. External callers should use
     * {@link #requestPoseReset(Pose2d)} instead.
     *
     * @param pose The desired field pose.
     */
    private void performReset(Pose2d pose) {
        Pose3d pose3d = new Pose3d(pose);

        // Reset QuestNav's internal pose
        quest.setPose(pose3d);

        // Reset drivetrain odometry
        swerve.setPose(pose);

        lastResetComplete = true;
        pendingReset = null;

        DriverStation.reportWarning(
            "[QuestNavController] Pose reset performed at: " + pose,
            false
        );
    }

    private void performSwerveOnlyReset(Pose2d pose) {
        swerve.setPose(pose);
        lastResetComplete = true;
    }

    /** @return The current drivetrain pose estimate. */
    public Pose2d getSwervePose() {
        return swerve.getPose();
    }

    /** @return The latest QuestNav‑reported pose. */
    public Pose2d getQuestPose() {
        return quest.getPose();
    }

    /** @return The QuestNav inputs*/
    public QuestNavInputs getInputs() {
        return quest.getInputs();
    }

    /** @return If the latest pose reset completed. */
    public boolean isResetComplete() {
        return lastResetComplete;
    }
}
