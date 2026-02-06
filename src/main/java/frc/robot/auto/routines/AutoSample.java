//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.robot.auto.routines;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.auto.AutoSequence;
import frc.robot.auto.AutoCapabilities;
import frc.robot.navigation.Nodes;

/**
 * AutoSample
 *
 * A simple example autonomous routine demonstrating how to use the
 * AutoSequence DSL with an {@link AutoCapabilities} object.
 *
 * <p>This routine is intentionally subsystem‑agnostic. All robot actions
 * (movement, intake, scoring, waiting) are provided through the capabilities
 * object created in RobotContainer.</p>
 */
public class AutoSample {

    /**
     * Builds the autonomous routine using the provided capabilities.
     *
     * @param caps            The set of available autonomous actions.
     * @param defaultSpeed    Default translational speed for movement.
     * @param defaultAngular  Default rotational speed for movement.
     *
     * @return A fully constructed autonomous command sequence.
     */
    public static Command build(
        AutoCapabilities caps,
        double defaultSpeed,
        double defaultAngular
    ) {

        return new AutoSequence(caps, defaultSpeed, defaultAngular)
            .moveTo(Nodes.Start.CENTER)
            .waitSeconds(2.0)
            .withSpeed(6.0).moveTo(Nodes.Start.LEFT)
            .score()   // only works if caps.score() is present
            .build();
    }
}
