//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS
//                         Programmed by Andrew :)
package frc.robot.commands.auto.routines;

import java.util.function.Consumer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.auto.AutoSequence;
import frc.robot.framework.CoordinatorRecord;
import frc.robot.framework.SubsystemsRecord;
import frc.robot.localization.nodes.Nodes;

public class AutoSubwayLeft {
    public static Command build(SubsystemsRecord record, CoordinatorRecord coordinator, Consumer<Pose2d> resetQuestPose, double MaxSpeed, double MaxAngularRate) {

        return new AutoSequence(record, coordinator, resetQuestPose, MaxSpeed, MaxAngularRate)
            .startTimer()
            .moveTo(Nodes.Start.LEFT)
            .moveThrough(Nodes.Midfield.LEFT_OVER_BUMP, 0.2)
            .moveTo(Nodes.Midfield.LEFT_LEFT_SUBWAY)
            .intakeDeploy()
            .moveThrough(Nodes.Midfield.RIGHT_LEFT_SUBWAY, 0.5)
             .intakeRetract()
            .moveThrough(Nodes.Midfield.LEFT_LEFT_SUBWAY, 0.5)
             .moveThrough(Nodes.Midfield.LEFT_OVER_BUMP, 0.2)
            .driveTo(Nodes.Start.LEFT)
            .shootUntil(18)
             .moveTo(Nodes.Depot.INTAKE_DEPOT)

            .build();
    }
}