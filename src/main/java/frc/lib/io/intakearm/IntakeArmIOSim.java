//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.lib.io.intakearm;

import edu.wpi.first.wpilibj.Timer;
import frc.lib.hardware.IntakeArmHardware;
import frc.lib.math.GearRatio;

/**
 * Simulation implementation of IntakeArmIO.
 */
public class IntakeArmIOSim implements IntakeArmIO {

    private final GearRatio ratio = IntakeArmHardware.RATIO;

    // Sim state (motor rotations, not degrees)
    private double bluMotorRot = 0.0;
    private double yelMotorRot = 0.0;

    // Target position (motor rotations)
    private double targetMotorRot = 0.0;

    // Simple physics constants
    private static final double kP = 8.0;              // how aggressively it moves toward target
    private static final double MAX_SPEED_RPS = 2.0;   // max motor rotation speed
    private static final double CURRENT_DRAW_A = 12.0; // fake current
    private static final double TEMP_C = 35.0;         // fake temperature

    private double lastTime = Timer.getFPGATimestamp();

    @Override
    public void updateInputs(IntakeArmInputs inputs) {
        double now = Timer.getFPGATimestamp();
        double dt = now - lastTime;
        lastTime = now;

        // --- Leader motor physics ---
        double error = targetMotorRot - bluMotorRot;
        double velocity = kP * error; // RPS

        // Clamp velocity
        velocity = Math.max(-MAX_SPEED_RPS, Math.min(MAX_SPEED_RPS, velocity));

        // Integrate position
        bluMotorRot += velocity * dt;

        // --- Follower motor (opposed alignment) ---
        yelMotorRot = -bluMotorRot;

        // --- Fill inputs ---
        inputs.bluMotorRot = bluMotorRot;
        inputs.yelMotorRot = yelMotorRot;

        inputs.bluPositionDeg = ratio.toOutput(bluMotorRot);
        inputs.yelPositionDeg = ratio.toOutput(yelMotorRot);

        inputs.bluCurrentA = CURRENT_DRAW_A;
        inputs.yelCurrentA = CURRENT_DRAW_A;

        inputs.bluTempC = TEMP_C;
        inputs.yelTempC = TEMP_C;
    }

    @Override
    public void setPositionDeg(double degrees) {
        targetMotorRot = ratio.toMotor(degrees);
    }

    @Override
    public void stop() {
        // Stop means: hold current position as the new target
        targetMotorRot = bluMotorRot;
    }
}
