//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.robot.commands.auto.routines;

import java.util.function.Consumer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.kShooter;
import frc.robot.commands.auto.AutoSequence;
import frc.robot.framework.CoordinatorRecord;
import frc.robot.framework.SubsystemsRecord;
import frc.robot.localization.nodes.Nodes;

public class AutoBlueDepot{
  /** Creates a new AutoBlueCenterRaymond. */
   public static Command build(SubsystemsRecord record, CoordinatorRecord coordinator, Consumer<Pose2d> resetQuestPose, double MaxSpeed, double MaxAngularRate) {

        return new AutoSequence(record, coordinator, resetQuestPose, MaxSpeed, MaxAngularRate)
            .startTimer()
            .shootRPMUntil(4, kShooter.kRPM.BUMP_RAYMOND)
            .driveTo(Nodes.Depot.INTAKE_DEPOT)
            .intakeDeploy()
            .withSpeed(MaxSpeed * 0.67)
            .driveDistance(0.85)
            .build();
    }

  }

