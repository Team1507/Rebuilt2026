//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.lib.io.agitator;

/**
 * Simulation implementation of AgitatorIO.
 */
public class AgitatorIOSim implements AgitatorIO {

    private double duty = 0.0;

    @Override
    public void updateInputs(AgitatorInputs inputs) {
        inputs.dutyCycle = duty;
        inputs.temperatureC = 25.0; // placeholder
        inputs.currentA = Math.abs(duty) * 5.0; // fake current draw
    }

    @Override
    public void run(double dutyCycle) {
        this.duty = dutyCycle;
    }

    @Override
    public void stop() {
        this.duty = 0.0;
    }
}
