//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.robot.auto;

import java.util.Optional;
import java.util.function.Function;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * AutoCapabilities
 *
 * A container describing which autonomous actions are available for a given
 * AutoSequence. RobotContainer constructs this object by wiring each action
 * factory to the appropriate subsystem.
 *
 * <p>Each field is optional. If an AutoSequence attempts to use an action that
 * is not provided, a clear exception is thrown to help students diagnose the
 * issue.</p>
 */
public record AutoCapabilities(
    Function<Pose2d, Command> moveTo,
    Optional<Supplier<Command>> intake,
    Optional<Supplier<Command>> score,
    Function<Double, Command> waitSeconds
) {}
