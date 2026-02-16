//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.lib.io.intakearm;

/**
 * Simulation implementation of IntakeArmIO.
 */
public class IntakeArmIOSim implements IntakeArmIO {

    private double positionDeg = 0.0;

    @Override
    public void updateInputs(IntakeArmInputs inputs) {
        inputs.bluPositionDeg = positionDeg;
        inputs.yelPositionDeg = positionDeg;

        inputs.bluMotorRot = positionDeg / 360.0;
        inputs.yelMotorRot = positionDeg / 360.0;

        inputs.bluCurrentA = 0.0;
        inputs.yelCurrentA = 0.0;

        inputs.bluTempC = 25.0;
        inputs.yelTempC = 25.0;
    }

    @Override
    public void setPositionDeg(double degrees) {
        this.positionDeg = degrees;
    }

    @Override
    public void stop() {
        // no-op
    }
}
