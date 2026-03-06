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
import frc.robot.framework.CoordinatorRecord;
import frc.robot.framework.SubsystemsRecord;
import frc.robot.localization.nodes.Nodes;


public class AutoBlueSubwayRight {
    public static Command build(SubsystemsRecord record, CoordinatorRecord coordinator, double MaxSpeed, double MaxAngularRate) {

        return new AutoSequence(record, coordinator, MaxSpeed, MaxAngularRate)
            
        //.moveTo(Nodes.Start.START_SUBWAY_RIGHT)
            .moveTo(Nodes.Midfield.RIGHT_OVER_BUMP)
            .moveTo(Nodes.Midfield.RIGHT_RIGHT_SUBWAY)
            .intakeDeploy()
            .moveTo(Nodes.Midfield.LEFT_RIGHT_SUBWAY)
            .intakeRetract()
            .moveThrough(Nodes.Midfield.RIGHT_RIGHT_SUBWAY, 0.5)
            .moveThrough(Nodes.Midfield.RIGHT_OVER_BUMP, 0.5)
            .moveTo(Nodes.Start.START_SUBWAY_RIGHT)
            .moveTo(Nodes.AllianceZoneBlue.BACK_RIGHT)
            .shoot()
            .moveTo(Nodes.Tower.APPROACH_RIGHT)
            .build();
    }
}