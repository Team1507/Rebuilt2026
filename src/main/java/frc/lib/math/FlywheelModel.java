//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.lib.math;

/**
 * Physics-based flywheel model using GearRatio.
 *
 * Public API uses RPM for convenience.
 * Internal math uses RPS for correctness.
 */
public class FlywheelModel {

    private final double inertia;        // kg·m² (wheel inertia)
    private final double motorKv;        // rad/s per volt
    private final double motorKt;        // N·m per amp
    private final double motorResistance;// ohms
    private final double frictionTorque; // N·m

    public FlywheelModel(
        double inertia,
        double motorKv,
        double motorKt,
        double motorResistance,
        double frictionTorque
    ) {
        this.inertia = inertia;
        this.motorKv = motorKv;
        this.motorKt = motorKt;
        this.motorResistance = motorResistance;
        this.frictionTorque = frictionTorque;
    }

    // -----------------------------
    // Helpers: RPM ↔ RPS
    // -----------------------------
    private static double rpmToRps(double rpm) {
        return rpm / 60.0;
    }

    private static double rpsToRpm(double rps) {
        return rps * 60.0;
    }

    // -----------------------------
    // Core physics
    // -----------------------------

    /** Compute wheel acceleration (RPS²) from wheel speed (RPM) and voltage. */
    public double computeAccelerationRPM(double wheelRPM, double voltage) {
        double wheelRPS = rpmToRps(wheelRPM);

        // Convert wheel speed to rad/s 
        double wheelRadPerSec = wheelRPS * 2 * Math.PI; 
        
        // Back-EMF 
        double backEmf = wheelRadPerSec / motorKv; 
        
        // Motor current 
        double current = (voltage - backEmf) / motorResistance; 
        
        // Motor torque 
        double motorTorque = current * motorKt; 
        
        // Subtract friction 
        double wheelTorque = motorTorque - frictionTorque * Math.signum(wheelRPS); 
        
        // Angular acceleration (rad/s²) 
        double accelRad = wheelTorque / inertia; 
        
        // Convert rad/s² → RPM/s 
        return rpsToRpm(accelRad / (2 * Math.PI));
    }

    /** Simulate one timestep of flywheel motion. */
    public double stepRPM(double wheelRPM, double voltage, double dt) {
        double accelRPM = computeAccelerationRPM(wheelRPM, voltage);
        return wheelRPM + accelRPM * dt;
    }
}
