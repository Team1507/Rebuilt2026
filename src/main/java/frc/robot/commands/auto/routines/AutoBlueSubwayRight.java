//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS
<<<<<<<< HEAD:src/main/java/frc/robot/commands/auto/routines/AutoBlueSubwayRight.java
//                         Programmed by Andrew :)
package frc.robot.auto.routines;
========

package frc.robot.commands.auto.routines;
>>>>>>>> bd0a7127c7dbd4a0dbca37b913926cb1a27e5c0d:src/main/java/frc/robot/commands/auto/routines/AutoSample.java

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.auto.AutoSequence;
import frc.robot.framework.SubsystemsRecord;
import frc.robot.localization.nodes.Nodes;

public class AutoBlueSubwayRight {

    public static Command build(SubsystemsRecord record, double maxSpeed, double maxAngularSpeed) {

        return new AutoSequence(record, maxSpeed, maxAngularSpeed)
            .moveTo(Nodes.Hub.START_SUBWAY_RIGHT)
            .moveTo(Nodes.Hub.RIGHT_OVER_BUMP)
            .moveTo(Nodes.Hub.RIGHT_SUBWAY)
            .intakeDeploy()
            .moveTo(Nodes.Hub.LEFT_SUBWAY)
            .intakeRetract()
            .moveTo(Nodes.Hub.RIGHT_SUBWAY)
            .moveTo(Nodes.Hub.BACK_RIGHT)
            .shoot()
            .build();
    }
}