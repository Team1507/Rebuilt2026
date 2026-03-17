//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;

import frc.robot.Constants.kQuest;
import frc.robot.framework.base.Subsystems1507;
import frc.robot.generated.questnav.PoseFrame;
import frc.lib.io.questnav.*;

public class QuestNavSubsystem extends Subsystems1507 {

    private final QuestNavIO io;
    private final QuestNavInputs inputs;

    private Pose2d latestPose = new Pose2d();
    private double latestTimestamp = 0.0;
    private boolean latestTracking = false;

    private double lastProcess = 0.0;
    private static final double PERIOD = 0.05;

    public QuestNavSubsystem(QuestNavIO io) {
        this.io = io;
        this.inputs = new QuestNavInputs();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);

        double now = Timer.getFPGATimestamp();
        if (now - lastProcess < PERIOD) return;
        lastProcess = now;

        if (inputs.frames.length == 0) {
            latestTracking = false;
            return;
        }

        PoseFrame frame = inputs.frames[inputs.frames.length - 1];
        latestTracking = frame.isTracking();

        if (!latestTracking) return;

        Pose3d questPose = frame.questPose3d();
        Pose3d robotPose = questPose.transformBy(kQuest.ROBOT_TO_QUEST.inverse());

        latestPose = new Pose2d(
            robotPose.getX(),
            robotPose.getY(),
            new Rotation2d(robotPose.getRotation().getZ())
        );

        latestTimestamp = frame.dataTimestamp();
    }

    // Public API
    public boolean isTracking() { return latestTracking; }
    public Pose2d getPose() { return latestPose; }
    public double getTimestamp() { return latestTimestamp; }

    public void setPose(Pose3d pose) {
        io.setPose(pose.transformBy(kQuest.ROBOT_TO_QUEST));
    }
}

