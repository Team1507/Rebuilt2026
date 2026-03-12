//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS
//                         Programmed by Andrew :)
package frc.robot.commands.auto.routines;




import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.auto.AutoSequence;
import frc.robot.framework.CoordinatorRecord;
import frc.robot.framework.SubsystemsRecord;
import frc.robot.localization.nodes.Nodes;


public class AutoSubway6inchLeft {
    public static Command build(SubsystemsRecord record, CoordinatorRecord coordinator, double MaxSpeed, double MaxAngularRate) {


        return new AutoSequence(record, coordinator, MaxSpeed, MaxAngularRate)
            .startTimer()
             .driveTo(Nodes.Start.LEFT)
            .moveThrough(Nodes.Midfield.LEFT_OVER_BUMP, 0.2)
            .intakeLow()
            .parallel(
                seq -> seq.withSpeed(MaxSpeed * 0.5).moveThrough(Nodes.Midfield.LEFT_LEFT_SUBWAY, 0.5),
                seq -> seq.intakeDeploy())
            .moveThrough(Nodes.Midfield.RIGHT_LEFT_SUBWAY, 0.5)


            .intakeLow()
            .parallel(
                seq -> seq.intakeRetract(),
                seq -> seq.moveThrough(Nodes.Midfield.LEFT_BEFORE_BUMP, 0.2))  
            .moveThrough(Nodes.Midfield.LEFT_OVER_BUMP, 0.2)
            .driveTo(Nodes.Start.LEFT)
            .shootUntil(15)
            .build();
    }
}
