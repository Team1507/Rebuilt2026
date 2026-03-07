//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.robot.commands.auto.routines;

import org.w3c.dom.traversal.NodeIterator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.kShooter;
import frc.robot.commands.auto.AutoSequence;
import frc.robot.framework.CoordinatorRecord;
import frc.robot.framework.SubsystemsRecord;
import frc.robot.localization.nodes.Nodes;

public class AutoTestQuest{
  /** Creates a new AutoBlueCenterRaymond. */
   public static Command build(SubsystemsRecord record, CoordinatorRecord coordinator, double MaxSpeed, double MaxAngularRate) {

        return new AutoSequence(record, coordinator, MaxSpeed, MaxAngularRate)
            .startTimer()
            .driveTo(Nodes.AllianceZoneBlue.CENTER)
            .build();
    }

  }

