//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.lib.io.intakearm;

/**
 * Container for intake arm sensor inputs.
 */
public class IntakeArmInputs {

    /** BLU arm position in mechanism degrees. */
    public double bluPositionDeg = 0.0;

    /** YEL arm position in mechanism degrees. */
    public double yelPositionDeg = 0.0;

    /** Velocity in deg/sec */
    public double bluVelocityDegPerSec = 0.0;
    public double yelVelocityDegPerSec = 0.0;

    /** applied voltages */
    public double bluAppliedVolts = 0.0;
    public double yelAppliedVolts = 0.0;

    /** Arm limit switches */
    public boolean bluReverseLimit = false;
    public boolean yelReverseLimit = false;

    /** Raw motor rotations. */
    public double bluMotorRot = 0.0;
    public double yelMotorRot = 0.0;

    /** Motor currents (amps). */
    public double bluCurrentA = 0.0;
    public double yelCurrentA = 0.0;

    /** Motor temperatures (Celsius). */
    public double bluTempC = 0.0;
    public double yelTempC = 0.0;
}
