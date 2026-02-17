//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.lib.io.intakeroller;

/**
 * Simulation implementation of IntakeRollerIO.
 */
public class IntakeRollerIOSim implements IntakeRollerIO {

    private double duty = 0.0;

    @Override
    public void updateInputs(IntakeRollerInputs inputs) {
        inputs.dutyCycle = duty;
        inputs.currentA = Math.abs(duty) * 4.0;
        inputs.temperatureC = 25.0;
    }

    @Override
    public void runDuty(double duty) {
        this.duty = duty;
    }

    @Override
    public void runPower(double power) {
        this.duty = power;
    }

    @Override
    public void stop() {
        this.duty = 0.0;
    }
}
