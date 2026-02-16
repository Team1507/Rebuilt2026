//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.lib.io.hopper;

/**
 * Simulation implementation of HopperIO.
 */
public class HopperIOSim implements HopperIO {

    private double positionDeg = 0.0;

    @Override
    public void updateInputs(HopperInputs inputs) {
        inputs.positionDeg = positionDeg;
        inputs.motorRot = positionDeg / 360.0;
        inputs.currentA = 0.0;
        inputs.temperatureC = 25.0;
    }

    @Override
    public void setPositionDeg(double degrees) {
        this.positionDeg = degrees;
    }

    @Override
    public void stop() {
        // no-op for sim
    }
}
