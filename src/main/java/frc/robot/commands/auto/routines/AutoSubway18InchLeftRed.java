// //  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
// //  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
// //  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
// //  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
// //  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
// //   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
// //                           TEAM 1507 WARLOCKS

// package frc.robot.commands.auto.routines;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.commands.auto.AutoSequence;
// import frc.robot.framework.CoordinatorRecord;
// import frc.robot.framework.SubsystemsRecord;
// import frc.robot.localization.nodes.Nodes;

// public class AutoSubway18InchLeftRed {
//     public static Command build(SubsystemsRecord record, CoordinatorRecord coordinator, double MaxSpeed, double MaxAngularRate) {

//         return new AutoSequence(record, coordinator, MaxSpeed, MaxAngularRate)

//             .startTimer()
//             .resetPose(Nodes.Start.Blue.LEFT_RED)
//             .moveThrough(Nodes.Start.Red.LEFT_OVER_BUMP, 0.2)
//             .intakeHigh()
//             .parallel(
//                 seq -> seq.moveThrough(Nodes.Midfield.LEFT_LEFT_SUBWAY_RED, 0.1),
//                 seq -> seq.intakeDeploy())
//             .withSpeed(MaxSpeed * 0.5).moveThrough(Nodes.Midfield.RIGHT_LEFT_SUBWAY_RED, 0.2)
//             .intakeHigh()
//             .parallel(
//                 seq -> seq.intakeRetract(),
//                 seq -> seq.moveThrough(Nodes.Start.Red.LEFT_BEFORE_BUMP, 0.5))
//             .moveThrough(Nodes.Start.Red.LEFT_OVER_BUMP, 0.2)  
//             .moveThrough(Nodes.Start.Red.LEFT, 0.2)
//             .driveTo(Nodes.Start.Blue.SHOOTING_SPOT_LEFT)
//             .withSpeed( MaxSpeed * 0.5).driveTo(Nodes.Start.Blue.SHOOTING_SPOT_RIGHT)
//             .pointToTarget(Nodes.Hub.CENTER)
//             .waitSeconds(0.5)
//             .pointToTarget(Nodes.Hub.RED_CENTER)
//             .shootUntil(10)
//             .changeHeading(Nodes.Midfield.Red.LEFT_TURN)
//             .moveThrough(Nodes.Midfield.Red.LEFT_OVER_BUMP2, 0.2)
//             .moveThrough(Nodes.Midfield.LEFT_LEFT_SUBWAY_RED,0.2)
//             .intakeHigh()
//             .parallel(
//                 seq -> seq.withSpeed(MaxSpeed *0.5).moveThrough(Nodes.Midfield.MIDDLE_18INCH_SUBWAY_RED, 0.1),
//                 seq -> seq.intakeDeploy())
//             .moveThrough(Nodes.Start.Red.LEFT_BEFORE_BUMP, 0.2)
//             .moveThrough(Nodes.Start.Red.LEFT_OVER_BUMP, 0.2)
//             .driveTo(Nodes.Start.Red.LEFT)
//             .waitSeconds(0.5)
//             .pointToTarget(Nodes.Hub.RED_CENTER)
//             .waitSeconds(0.5)
//             .shootUntil(19.99)
//             .build();
//     }
// }
