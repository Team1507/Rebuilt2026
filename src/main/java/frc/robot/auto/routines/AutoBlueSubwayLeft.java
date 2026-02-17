//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS
//                         Programmed by Andrew :)
package frc.robot.auto.routines;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.auto.AutoSequence;
import frc.robot.utilities.SubsystemsRecord;
import frc.robot.navigation.Nodes;

public class AutoBlueSubwayLeft {

    public static Command build(SubsystemsRecord record, double maxSpeed, double maxAngularSpeed) {

        return new AutoSequence(record, maxSpeed, maxAngularSpeed)
            .moveTo(Nodes.Hub.START_SUBWAY_LEFT)
            .moveTo(Nodes.Hub.LEFT_OVER_BUMP)
            .moveTo(Nodes.Hub.LEFT_SUBWAY)
            .intakeDeploy()
            .moveTo(Nodes.Hub.RIGHT_SUBWAY)
            .intakeRetract()
            .moveTo(Nodes.Hub.LEFT_SUBWAY)
            .moveTo(Nodes.Hub.BACK_LEFT)
            .shoot()
            .build();
    }
}