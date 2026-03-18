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

public class AutoSubway18Inch {
    public static Command build(SubsystemsRecord record, CoordinatorRecord coordinator, double MaxSpeed, double MaxAngularRate) {

        return new AutoSequence(record, coordinator, MaxSpeed, MaxAngularRate)

            .startTimer()
            .resetPose(Nodes.Start.Blue.RIGHT)
            .moveThrough(Nodes.Midfield.RIGHT_OVER_BUMP, 0.2)
            .intakeHigh()
            .parallel(
                seq -> seq.moveThrough(Nodes.Midfield.RIGHT_RIGHT_SUBWAY, 0.1),
                seq -> seq.intakeDeploy())
            .withSpeed(MaxSpeed * 0.8).moveThrough(Nodes.Midfield.LEFT_RIGHT_SUBWAY, 0.2)
            .intakeHigh()
            .parallel(
                seq -> seq.intakeRetract(),
                seq -> seq.moveThrough(Nodes.Midfield.RIGHT_BEFORE_BUMP, 0.5))
                .moveThrough(Nodes.Midfield.RIGHT_OVER_BUMP, 0.2)  
            .moveThrough(Nodes.Start.Blue.RIGHT, 0.2)
            .waitSeconds(0.5)
            .moveThrough(Nodes.Start.Blue.RIGHT, 0.2)
            .withSpeed( MaxSpeed * 0.7).driveTo(Nodes.Start.Blue.SHOOTING_SPOT_RIGHT)
            .pointToTarget(Nodes.Hub.CENTER)
            .shootUntil(10)
            .moveThrough(Nodes.Start.Blue.RIGHT, 0.1)
            .changeHeading(Nodes.Midfield.RIGHT_TURN)
            .moveThrough(Nodes.Midfield.RIGHT_TURN, 0.2)
            .moveThrough(Nodes.Midfield.RIGHT_OVER_BUMP2, 0.2)
            .moveThrough(Nodes.Midfield.RIGHT_RIGHT_SUBWAY,0.2)
            .intakeHigh()
            .parallel(
                seq -> seq.withSpeed(MaxSpeed *0.8).moveThrough(Nodes.Midfield.MIDDLE_18INCH_SUBWAY, 0.1),
                seq -> seq.intakeDeploy())            .moveThrough(Nodes.Midfield.LEFT_RIGHT_SUBWAY, 0.2)
                .moveThrough(Nodes.Midfield.RIGHT_BEFORE_BUMP, 0.2)
                .moveThrough(Nodes.Midfield.RIGHT_OVER_BUMP, 0.2)
                .driveTo(Nodes.Start.Blue.RIGHT)
            .waitSeconds(0.5)
            .withSpeed( MaxSpeed * 0.5).driveTo(Nodes.Start.Blue.SHOOTING_SPOT_RIGHT)
            .pointToTarget(Nodes.Hub.CENTER)
            .shootUntil(19.99)
            .build();
    }
}
