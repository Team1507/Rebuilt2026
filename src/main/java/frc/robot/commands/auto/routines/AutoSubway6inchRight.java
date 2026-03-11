//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.robot.commands.auto.routines;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.auto.AutoSequence;
import frc.robot.framework.CoordinatorRecord;
import frc.robot.framework.SubsystemsRecord;
import frc.robot.localization.nodes.Nodes;
import frc.robot.Constants.kShooter;


public class AutoSubway6inchRight {
    public static Command build(SubsystemsRecord record, CoordinatorRecord coordinator, double MaxSpeed, double MaxAngularRate) {

        return new AutoSequence(record, coordinator, MaxSpeed, MaxAngularRate)

            .startTimer()
             //.driveTo(Nodes.Start.RIGHT)
            .moveThrough(Nodes.Midfield.RIGHT_OVER_BUMP, 0.2)
            .intakeLow()
            .parallel(
                seq -> seq.withSpeed(MaxSpeed * 0.5).moveThrough(Nodes.Midfield.RIGHT_RIGHT_SUBWAY, 0.2),
                seq -> seq.intakeDeploy())
            .withSpeed(MaxSpeed * 0.5).moveThrough(Nodes.Midfield.LEFT_RIGHT_SUBWAY, 0.2)
           
            .intakeHigh()
            .parallel(
                seq -> seq.intakeRetract(),
                seq -> seq.moveThrough(Nodes.Midfield.RIGHT_BEFORE_BUMP, 0.5))  
            .driveTo(Nodes.Start.RIGHT)
            .shootRPMUntil(15, kShooter.kRPM.BUMP_RAYMOND)
            //.shootUntil(15)
            //second time
            //  .moveThrough(Nodes.Midfield.RIGHT_OVER_BUMP, 0.2)
            // .intakeHigh()
            // .parallel(
            //     seq -> seq.withSpeed(MaxSpeed * 0.5).moveThrough(Nodes.Midfield.RIGHT_RIGHT_SUBWAY, 0.5),
            //     seq -> seq.intakeDeploy())
            // .moveThrough(Nodes.Midfield.LEFT_RIGHT_SUBWAY, 0.2)
            // .intakeLow()
            // .parallel(
            //     seq -> seq.intakeRetract(),
            //     seq -> seq.moveThrough(Nodes.Midfield.RIGHT_OVER_BUMP, 0.2))  
            // .driveTo(Nodes.Start.RIGHT)
            // .shootRPMUntil(20, kShooter.kRPM.BUMP_RAYMOND)
            .build();
    }
}