//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.robot.localization.quest;

import java.util.OptionalDouble;
import java.util.OptionalInt;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kQuest;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utilities.Telemetry;

public class QuestNavSubsystem extends SubsystemBase {

    private final CommandSwerveDrivetrain drivetrain;
    private final QuestNav questNav = new QuestNav();
    private final Telemetry telemetry;

    // Throttle QuestNav processing to 20 Hz
    private double lastProcessTime = 0.0;
    private static final double PROCESS_PERIOD = 0.05; // 50 ms

    public QuestNavSubsystem(CommandSwerveDrivetrain drivetrain, Telemetry telemetry) {
        this.drivetrain = drivetrain;
        this.telemetry = telemetry;

        // Register QuestNav as a vision source
        telemetry.registerVisionPoseSource("QuestNav");
        telemetry.registerVisionPoseSource("QuestNav-Raw");
    }

    @Override
    public void periodic() {

        questNav.commandPeriodic();

        double now = Timer.getFPGATimestamp();
        if (now - lastProcessTime < PROCESS_PERIOD) {
            return;
        }
        lastProcessTime = now;

        // Log connection state every cycle
        telemetry.logQuestNavConnected(questNav.isConnected());

        PoseFrame[] frames = questNav.getAllUnreadPoseFrames();
        if (frames.length == 0) {
            telemetry.logQuestNavTracking(false);
            return;
        }

        // Only process the latest frame
        PoseFrame frame = frames[frames.length - 1];

        // Log tracking state
        telemetry.logQuestNavTracking(frame.isTracking());

        if (!frame.isTracking()) {
            telemetry.logQuestNavCorrectedPose(new Pose2d());
            return;
        }

        Pose3d questPose = frame.questPose3d();
        double frameTimestamp = frame.dataTimestamp();

        // Log raw QuestNav pose
        telemetry.logQuestNavRawPose(questPose);

        // Log frame timestamp
        telemetry.logQuestNavFrameTimestamp(frameTimestamp);

        // Compute corrected robot pose
        Pose3d robotPose = questPose.transformBy(kQuest.ROBOT_TO_QUEST.inverse());
        Rotation2d pigeonHeading = drivetrain.getHeading();

        Pose2d correctedPose = new Pose2d(
            robotPose.getX(),
            robotPose.getY(),
            pigeonHeading
        );

        // Log corrected pose
        telemetry.logQuestNavCorrectedPose(correctedPose);

        // Log latency (ms)
        double latencyMs = (now - frameTimestamp) * 1000.0;
        telemetry.logQuestNavLatency(latencyMs);

        // Log battery %
        questNav.getBatteryPercent().ifPresent(
            pct -> telemetry.logQuestNavBattery(pct)
        );

        // Log app timestamp
        questNav.getAppTimestamp().ifPresent(
            appTs -> telemetry.logQuestNavAppTimestamp(appTs)
        );

        // Feed corrected pose into drivetrain estimator
        drivetrain.addVisionMeasurement(correctedPose, frameTimestamp, kQuest.STD_DEVS);
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
