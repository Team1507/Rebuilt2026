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
            .moveTo(Nodes.Hub.START_SUBWAY_RIGHT)
            .moveThrough(Nodes.Hub.RIGHT_OVER_BUMP, 0.5)
            .moveTo(Nodes.Hub.RIGHT_SUBWAY)
            //.intakeDeploy()
            .moveTo(Nodes.Hub.LEFT_SUBWAY)
            //.intakeRetract()
            .moveThrough(Nodes.Hub.RIGHT_SUBWAY, 1.0)
            .moveThrough(Nodes.Hub.BACK_RIGHT, 0.5)
            //.shoot()
            .build();
    }
}