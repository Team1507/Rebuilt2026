//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.lib.io.questnav;

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.generated.questnav.QuestNav;

public class QuestNavIOReal implements QuestNavIO {

    private final QuestNav quest = new QuestNav();

    @Override
    public void updateInputs(QuestNavInputs inputs) {
        quest.commandPeriodic();

        inputs.frames = quest.getAllUnreadPoseFrames();
        inputs.connected = quest.isConnected();
        inputs.batteryPercent = quest.getBatteryPercent();
        inputs.appTimestamp = quest.getAppTimestamp();
    }

    @Override
    public void setPose(Pose3d pose) {
        quest.setPose(pose);
    }
}
