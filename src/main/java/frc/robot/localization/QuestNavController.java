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
import frc.robot.Constants.kQuest;
import frc.robot.framework.base.Subsystems1507;

public class QuestNavController extends Subsystems1507 {

    private final SwerveSubsystem swerve;
    private final QuestNavSubsystem quest;

    private double lastProcess = 0.0;
    private static final double PERIOD = 0.05;

    public QuestNavController(SwerveSubsystem swerve, QuestNavSubsystem quest) {
        this.swerve = swerve;
        this.quest = quest;
    }

    @Override
    public void periodic() {

        double now = Timer.getFPGATimestamp();
        if (now - lastProcess < PERIOD) return;
        lastProcess = now;

        boolean enabled = DriverStation.isEnabled();
        boolean tracking = quest.isTracking();

        Pose2d questPose = quest.getPose();
        double questTs = quest.getTimestamp();

        // DISABLED → seed drivetrain + QuestNav
        if (!enabled) {
            if (tracking) {
                swerve.setPose(questPose);
                quest.setPose(new Pose3d(questPose));
            }
            return;
        }

        // ENABLED → QuestNav is the ONLY vision source
        if (tracking) {
            swerve.addVisionMeasurement(
                questPose,
                questTs,
                kQuest.STD_DEVS
            );
        }
    }

    public void resetPose(Pose2d pose) {
        quest.setPose(new Pose3d(pose));
        swerve.setPose(pose);
    }

    public Pose2d getSwervePose() { return swerve.getPose(); }
    public Pose2d getQuestPose() { return quest.getPose(); }
}
