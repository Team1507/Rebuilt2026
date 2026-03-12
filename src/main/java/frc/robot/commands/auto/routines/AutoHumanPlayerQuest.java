//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.robot.commands.auto.routines;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.kShooter;
import frc.robot.commands.auto.AutoSequence;
import frc.robot.framework.CoordinatorRecord;
import frc.robot.framework.SubsystemsRecord;
import frc.robot.localization.nodes.Nodes;

public class AutoHumanPlayerQuest{
  /** Creates a new AutoBlueCenterRaymond. */
   public static Command build(SubsystemsRecord record, CoordinatorRecord coordinator, double MaxSpeed, double MaxAngularRate) {

        return new AutoSequence(record, coordinator, MaxSpeed, MaxAngularRate)
            .startTimer()
            .shootRPMUntil(3.5, kShooter.kRPM.BUMP_RAYMOND)
            .driveTo(Nodes.Outpost.RIGHT_APPROACH_POINT_QUEST)
            .intakeDeploy()
            .withSpeed(MaxSpeed * 0.7) .driveDistance(0.15)
            .waitSeconds(1.5)
            .withSpeed(MaxSpeed * .7).moveThrough(Nodes.Outpost.RIGHT_APPROACH_POINT, 0.1)
            .parallel(  
              seq -> seq.intakeRetract(),
              seq -> seq.driveTo(Nodes.Start.RIGHT))
            .shootRPMUntil(19.5, kShooter.kRPM.BUMP_RAYMOND)
            .build();
    }

  }

