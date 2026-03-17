//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.lib.io.intakeroller.IntakeRollerIO;
import frc.lib.io.intakeroller.IntakeRollerInputs;

/**
 * Thin, IO-based intake roller subsystem.
 */
public class IntakeRollerSubsystem extends SubsystemBase {

    private final IntakeRollerIO io;
    private final IntakeRollerInputs inputs;

    public IntakeRollerSubsystem(IntakeRollerIO io) {
        this.io = io;
        this.inputs = new IntakeRollerInputs();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }

    public void run() {
        io.runDuty(inputs.cmdDutyCycle);
    }

    public void run(double duty) {
        io.runDuty(duty);
    }

    public void runPower(double power) {
        io.runPower(power);
    }

    public void stop() {
        io.stop();
    }

    public IntakeRollerInputs getInputs() {
        return inputs;
    }

    public void setDutyCycle(double duty) {
        inputs.cmdDutyCycle = duty;
    }

    public double getDutyCycle() {
        return inputs.dutyCycle;
    }

    /**
     * Increments the cmdDutyCycle by 0.1 if the value
     * is less than 0.8
     */
    public void incrementDutyCycle() {
        if(inputs.cmdDutyCycle < 0.8) {
            inputs.cmdDutyCycle += 0.1;
        }
    }
}
