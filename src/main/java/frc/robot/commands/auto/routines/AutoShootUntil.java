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

public class AutoShootUntil {
    public static Command build(SubsystemsRecord record, double maxSpeed, double maxAngularSpeed) {

        return new AutoSequence(record, maxSpeed, maxAngularSpeed)
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

            .startLog(Nodes.AllianceZoneBlue.BACK_RIGHT)
            .parallel(
                seq -> seq.recordLog(),
                seq -> seq.moveTo(Nodes.AllianceZoneBlue.BACK_RIGHT)
            )
            .endLog()

            .pointToTarget()
            .shootUntil(7.0)

            .startLog(Nodes.Tower.APPROACH_RIGHT)
            .parallel(
                seq -> seq.recordLog(),
                seq -> seq.moveTo(Nodes.Tower.APPROACH_RIGHT)
            )
            .endLog()

            .startLog(Nodes.Tower.CLIMB)
            .parallel(
                seq -> seq.recordLog(),
                seq -> seq.moveTo(Nodes.Tower.CLIMB)
            )
            .endLog()

            .build();
    }
}
