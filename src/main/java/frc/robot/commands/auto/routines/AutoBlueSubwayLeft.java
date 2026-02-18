//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS
//                         Programmed by Andrew :)
package frc.robot.commands.auto.routines;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.auto.AutoSequence;
import frc.robot.framework.SubsystemsRecord;
import frc.robot.localization.nodes.Nodes;

public class AutoBlueSubwayLeft {

    public static Command build(SubsystemsRecord record, double maxSpeed, double maxAngularSpeed) {

        return new AutoSequence(record, maxSpeed, maxAngularSpeed)
            //.moveTo(Nodes.Start.START_SUBWAY_LEFT)
            .moveTo(Nodes.Midfield.LEFT_OVER_BUMP)
            .moveTo(Nodes.Midfield.LEFT_LEFT_SUBWAY)
            .intakeDeploy()
            .moveTo(Nodes.Midfield.RIGHT_LEFT_SUBWAY)
            .intakeRetract()
            .moveTo(Nodes.Midfield.LEFT_LEFT_SUBWAY)
            .moveTo(Nodes.AllianceZoneBlue.BACK_LEFT)
            .shoot()
            .moveTo(Nodes.Tower.APPROACH_LEFT)
            .build();
    }
}