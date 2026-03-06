//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.lib.core.util;

import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.ConcurrentModificationException;
import java.util.function.BooleanSupplier;

/**
 * Lightweight, allocation-free command builder.
 *
 * This class is designed for IO-based subsystems and minimizes
 * cyclic load by:
 *  - storing static lambdas (no per-loop allocations)
 *  - avoiding FunctionalCommand wrappers
 *  - avoiding subclass boilerplate
 *  - avoiding repeated lambda creation
 *
 * Commands created with this builder are extremely cheap to run.
 */
public class CommandBuilder extends Command {

    private Runnable onInit = () -> {};
    private Runnable onExecute = () -> {};
    private BooleanConsumer onEnd = interrupted -> {};
    private BooleanSupplier isFinished = () -> false;

    /**
     * Creates a new CommandBuilder with subsystem requirements.
     */
    public CommandBuilder(Subsystem... requirements) {
        addRequirements(requirements);
    }

    /**
     * Optional: give the command a name for debugging.
     */
    public CommandBuilder named(String name) {
        setName(name);
        return this;
    }

    /**
     * Called once when the command is scheduled.
     */
    public CommandBuilder onInitialize(Runnable r) {
        if (isScheduled()) throw new ConcurrentModificationException(
            "Cannot modify a running CommandBuilder"
        );
        this.onInit = r;
        return this;
    }

    /**
     * Called every scheduler cycle.
     */
    public CommandBuilder onExecute(Runnable r) {
        if (isScheduled()) throw new ConcurrentModificationException(
            "Cannot modify a running CommandBuilder"
        );
        this.onExecute = r;
        return this;
    }

    /**
     * Called when the command ends (normal or interrupted).
     */
    public CommandBuilder onEnd(Runnable r) {
        return onEnd(interrupted -> r.run());
    }

    /**
     * Called when the command ends (normal or interrupted).
     */
    public CommandBuilder onEnd(BooleanConsumer r) {
        if (isScheduled()) throw new ConcurrentModificationException(
            "Cannot modify a running CommandBuilder"
        );
        this.onEnd = r;
        return this;
    }

    /**
     * Sets the command's termination condition.
     */
    public CommandBuilder isFinished(BooleanSupplier s) {
        if (isScheduled()) throw new ConcurrentModificationException(
            "Cannot modify a running CommandBuilder"
        );
        this.isFinished = s;
        return this;
    }

    /**
     * Convenience overload for constant-finish commands.
     */
    public CommandBuilder isFinished(boolean finished) {
        return isFinished(() -> finished);
    }

    // WPILib lifecycle
    @Override public void initialize() { onInit.run(); }
    @Override public void execute() { onExecute.run(); }
    @Override public void end(boolean interrupted) { onEnd.accept(interrupted); }
    @Override public boolean isFinished() { return isFinished.getAsBoolean(); }
}
