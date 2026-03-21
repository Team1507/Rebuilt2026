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

public class AutoRushToSubway {
    public static Command build(SubsystemsRecord record, CoordinatorRecord coordinator, double MaxSpeed, double MaxAngularRate) {

        return new AutoSequence(record, coordinator, MaxSpeed, MaxAngularRate)

            .resetPose(Nodes.Start.RIGHT)
            .startTimer()
            .moveThrough(Nodes.Midfield.RIGHT_OVER_BUMP, 0.2)
            .moveThrough(Nodes.Midfield.MIDDLE_PICKUP_SUBWAY, 0.2)
            .moveThrough(Nodes.Midfield.RUSH_MIDFIELD_BUMP, 0.2)
            .driveTo(Nodes.Midfield.LEFT_RUSH_SUBWAY)
            .intakeHigh()
            .parallel(
                seq -> seq.moveThrough(Nodes.Midfield.LEFT_LEFT_SUBWAY, 0.1),
                seq -> seq.intakeDeploy())
            .withSpeed(MaxSpeed *0.5).moveThrough(Nodes.Midfield.RIGHT_RIGHT_SUBWAY_RUSH, 0.2)
            .intakeRetract()
            .intakeLow()
            .moveThrough(Nodes.Midfield.RIGHT_BEFORE_BUMP, 0.5)
            .moveThrough(Nodes.Midfield.RIGHT_OVER_BUMP, 0.2)
            .driveTo(Nodes.Start.SHOOTING_SPOT_RIGHT)
            .pointToTarget(Nodes.Hub.CENTER)
            .shootUntil(10.25)
            .intakeShoot()
            .shootUntil(14.0)

            

            .build();
    }
}