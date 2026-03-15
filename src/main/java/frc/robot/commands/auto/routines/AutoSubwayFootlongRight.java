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

public class AutoSubwayFootlongRight {
    public static Command build(SubsystemsRecord record, CoordinatorRecord coordinator, double MaxSpeed, double MaxAngularRate) {

        return new AutoSequence(record, coordinator, MaxSpeed, MaxAngularRate)
            .startTimer()
            .driveTo(Nodes.Start.Blue.RIGHT)
            .moveThrough(Nodes.Midfield.RIGHT_OVER_BUMP, 0.2)
            .intakeHigh()
            .parallel(
                seq -> seq.withSpeed(MaxSpeed * 0.5).moveThrough(Nodes.Midfield.RIGHT_RIGHT_SUBWAY, 0.5),
                seq -> seq.intakeDeploy())
            .moveThrough(Nodes.Midfield.LEFT_RIGHT_SUBWAY, 0.7)
            .moveThrough(Nodes.Midfield.LEFT_FOOTLONG_SUBWAY, 0.2)
           
            .intakeLow()
            .parallel(
                seq -> seq.intakeRetract(),
                seq -> seq.moveThrough(Nodes.Midfield.RIGHT_BEFORE_BUMP, 0.2))
            .moveThrough(Nodes.Midfield.RIGHT_OVER_BUMP, 0.2)
            .driveTo(Nodes.Start.Blue.RIGHT)
            .shootUntil(19.5)
            .build();
    }
}