//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.robot.commands.auto.routines;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.auto.AutoSequence;
import frc.robot.framework.SubsystemsRecord;
import frc.robot.localization.nodes.Nodes;

public class AutoSample {

    public static Command build(SubsystemsRecord record, double maxSpeed, double maxAngularSpeed) {

        return new AutoSequence(record, maxSpeed, maxAngularSpeed)
            .moveTo(Nodes.Hub.RIGHT_SUBWAY)
            .intakeDeploy()
            .moveTo(Nodes.Hub.LEFT_SUBWAY)
            .intakeRetract()
            .moveTo(Nodes.Hub.RIGHT_SUBWAY)
            .moveTo(Nodes.Hub.BACK_RIGHT)
            .shoot()
            .build();
    }
}