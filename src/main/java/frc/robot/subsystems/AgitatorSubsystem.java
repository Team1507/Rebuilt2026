//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.lib.io.agitator.AgitatorIO;
import frc.lib.io.agitator.AgitatorInputs;

/**
 * Thin, IO-based agitator subsystem.
 */
public class AgitatorSubsystem extends SubsystemBase {

    private final AgitatorIO io;
    private final AgitatorInputs inputs = new AgitatorInputs();

    public AgitatorSubsystem(AgitatorIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        // Optional: log telemetry
    }

    public void run(double dutyCycle) {
        io.run(dutyCycle);
    }

    public void stop() {
        io.stop();
    }

    public double getDutyCycle() {
        return inputs.dutyCycle;
    }
}
