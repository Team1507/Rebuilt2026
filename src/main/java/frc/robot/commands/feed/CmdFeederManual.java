package frc.robot.commands.feed;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FeederSubsystem;
import java.util.function.Supplier;

public class CmdFeederManual extends Command {
    private final FeederSubsystem feederSubsystem;
    private final Supplier<Double> rpmSupplier;

    public CmdFeederManual(FeederSubsystem feederSubsystem, Supplier<Double> rpmSupplier) {
        this.feederSubsystem = feederSubsystem;
        this.rpmSupplier = rpmSupplier;
        
        addRequirements(feederSubsystem);
    }

    @Override
    public void execute() {
        feederSubsystem.run(rpmSupplier.get());
    }

    @Override
    public void end(boolean interrupted) {
        feederSubsystem.stopMotor();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}