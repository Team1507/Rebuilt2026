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

public class AutoShootUntil {
    public static Command build(SubsystemsRecord record, CoordinatorRecord coordinator, double MaxSpeed, double MaxAngularRate) {

        return new AutoSequence(record, coordinator, MaxSpeed, MaxAngularRate)
            .startTimer()
            //.moveTo(Nodes.Start.START_SUBWAY_RIGHT)
            // .moveThrough(Nodes.Midfield.RIGHT_OVER_BUMP, 0.5)
            // .moveThrough(Nodes.Midfield.RIGHT_RIGHT_SUBWAY, 0.5)
            // .intakeDeploy()
            // .moveThrough(Nodes.Midfield.LEFT_RIGHT_SUBWAY, 1.0)
            // .intakeRetract()
            // .moveThrough(Nodes.Midfield.RIGHT_RIGHT_SUBWAY, 0.5)
            // .moveThrough(Nodes.Midfield.RIGHT_OVER_BUMP, 0.5)
            // .moveThrough(Nodes.Start.START_SUBWAY_RIGHT, 0.5)
            .moveTo(Nodes.AllianceZoneBlue.BACK_RIGHT)
            .pointToShoot()
            .shootUntil(7.0)
            .moveTo(Nodes.Tower.APPROACH_RIGHT)
            .moveTo(Nodes.Tower.CLIMB)
            .build();
    }
}
