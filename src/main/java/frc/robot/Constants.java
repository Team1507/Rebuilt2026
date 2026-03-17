//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

import com.ctre.phoenix6.signals.ReverseLimitTypeValue;

import frc.lib.core.math.FlywheelModel;
import frc.lib.core.util.MotorConfig;
import frc.lib.core.util.MotorConfig.ControlMode;
import frc.lib.core.util.MotorConfig.GravityType;
import frc.robot.generated.ctre.TunerConstants;

/**
 * Central location for all robot-wide constants.
 * 
 * Subsystems should reference values from here rather than hardcoding
 * numbers in multiple places. This improves readability, tuning, and
 * maintainability across the project.
 */
public class Constants {

    // ╔═══════════════════════════════════════════════════════════════╗
    // ║                      AGITATOR CONSTANTS                       ║
    // ║               (Arcane Material Stirring Engine)               ║
    // ╚═══════════════════════════════════════════════════════════════╝
    public static final class kAgitator {

        /**
         * MotorConfig now contains ONLY tuning values.
         * Hardware (CAN ID) lives in AgitatorHardware.
         */
        public static final MotorConfig CONFIG =
            MotorConfig.builder()
                .inverted(true)
                .withVoltageLimits(3.5, -3.5)
                .withStatorCurrentLimit(Amps.of(80))
                .build();


        /** Duty cycles for agitator behavior. */
        public static final double AGITATE_TO_SHOOTER_DUTY = 0.7;
        public static final double AGITATE_TO_INTAKE_DUTY = -0.3;
    }

    // ╔═══════════════════════════════════════════════════════════════╗
    // ║                       CLIMBER CONSTANTS                       ║
    // ║                (Ether‑Lift Ascension Mechanism)               ║
    // ╚═══════════════════════════════════════════════════════════════╝
    public static final class kClimber {

        /** Mechanism setpoints (in mechanism units, not motor rotations). */
        public static final double UP = 0.0; //if climber is zero wile down set 1.6
        public static final double DOWN = -1.6;
 
        /** Robot-relative aliases for convenience. */
        public static final double ROBOT_UP = DOWN;
        public static final double ROBOT_DOWN = UP;

        /**
         * MotorConfig now contains ONLY tuning values.
         * Hardware (CAN ID, servo port, gear ratio) lives in ClimberHardware.
         */
        public static final MotorConfig CONFIG_SLOT0 =
            MotorConfig.builder(ControlMode.MOTION_MAGIC)
                .withPID(0.11, 0.0, 0.0)
                .withFeedforward(0.245, 0.09375, 0.0)
                .withGravity(0.0, GravityType.CONSTANT)
                .withVoltageLimits(12.0, -12.0)
                .build();

        public static final MotorConfig CONFIG_SLOT1 =
            MotorConfig.builder(ControlMode.MOTION_MAGIC)
                .slot(1)
                .withPID(0.11, 0.0, 0.0)
                .withFeedforward(0.245, 0.09375, 0.0)
                .withGravity(0.8, GravityType.CONSTANT)
                .build();
    }

    // ╔═══════════════════════════════════════════════════════════════╗
    // ║                       FEEDER CONSTANTS                        ║
    // ║             (Mana‑Driven Material Uplift System)              ║
    // ╚═══════════════════════════════════════════════════════════════╝
    public static final class kFeeder {

        public static final double FEED_RPM = 500.0;
        public static final double VOMIT_RPM = -250.0;

        public static final MotorConfig BLU_CONFIG =
            MotorConfig.builder(ControlMode.VELOCITY)
                .inverted(true)
                .withPID(0.11, 0.0, 0.0)
                .withFeedforward(0.42, 0.09931, 0.0)
                .withVoltageLimits(12.0, -12.0)
                .withStatorCurrentLimit(Amps.of(80))
                .build();

        public static final MotorConfig YEL_CONFIG =
            MotorConfig.builder(ControlMode.VELOCITY)
                .inverted(false)
                .withPID(0.11, 0.0, 0.02)
                .withFeedforward(0.42, 0.09931, 0.0)
                .withVoltageLimits(12.0, -12.0)
                .withStatorCurrentLimit(Amps.of(80))
                .build();
    }

    // ╔═══════════════════════════════════════════════════════════════╗
    // ║                       HOPPER CONSTANTS                        ║
    // ║                (Arcane Material Buffer Chamber)               ║
    // ╚═══════════════════════════════════════════════════════════════╝
    public static final class kHopper {

        /** Hopper target positions (in degrees). */ 
        public static final double RETRACTED_POS = 0.0; 
        public static final double EXTENDED_POS = 12.0;
        public static final double SAFE_EXTENDED = 10.0;
        public static final double MAX_POS = 12.1;

        public static final double MANUAL_POSITIVE_POWER = -0.2;
        public static final double MANUAL_NEGATIVE_POWER = 0.2;

        /**
         * MotorConfig now contains ONLY tuning values.
         * Hardware (CAN ID, gear ratio) lives in HopperHardware.
         */
        public static final MotorConfig CONFIG =
            MotorConfig.builder(ControlMode.POSITION)
                .withPID(0.11, 0.0, 0.02)
                .withVoltageLimits(8.0, -8.0)
                .withStatorCurrentLimit(Amps.of(25))
                .withBrake()
                .build();
    }

    // ╔═══════════════════════════════════════════════════════════════╗
    // ║                       INTAKE CONSTANTS                        ║
    // ║                 (Soul‑Siphon Acquisition Arm)                 ║
    // ╚═══════════════════════════════════════════════════════════════╝
    public static final class kIntake {

        public static final double INTAKE_ROLLER_DUTY_LOW = 0.3;
        public static final double INTAKE_ROLLER_DUTY_HIGH = 0.6;
        public static final double INTAKE_ROLLER_DUTY_IDLE = 0.1;
        public static final double OUTTAKE_ROLLER_DUTY = -0.35;
        public static final double INTAKE_AUTO_ROLLER_DUTY = 0.8;
        
        public static final MotorConfig ROLLER_CONFIG =
            MotorConfig.builder(ControlMode.DUTY_CYCLE)
                .withVoltageLimits(7.0, -7.0)
                .withStatorCurrentLimit(Amps.of(40))
                .build();

        public static final class kArm {

            public static final double MAX_ANGLE_DEGREES = 140.0;
            public static final double MIN_ANGLE_DEGREES = 0.0;
            public static final double DEPLOYED_ANGLE_DEGREES = 138.0;
            public static final double RETRACTED_ANGLE_DEGREES = 82.0;

            public static final double MANUAL_POSITIVE_POWER = 0.4;
            public static final double MANUAL_NEGATIVE_POWER = -0.4;

            public static final MotorConfig BLU_CONFIG =
                MotorConfig.builder(ControlMode.POSITION)
                    .inverted(false)
                    .withPID(0.5, 0.0, 0.0)
                    .withGravity(0.1, GravityType.COSINE)
                    .withReverseLimit(true, true, 0.0) // enable, autoset, reset to 0.0 
                    .reverseLimitType(ReverseLimitTypeValue.NormallyOpen)
                    .withVoltageLimits(8, -8)
                    .withStatorCurrentLimit(Amps.of(30.0))
                    .withBrake()
                    .build();

            public static final MotorConfig YEL_CONFIG =
                MotorConfig.builder(ControlMode.POSITION)
                    .inverted(true)
                    .withPID(0.5, 0.0, 0.0)
                    .withGravity(0.1, GravityType.COSINE)
                    .withReverseLimit(true, true, 0.0) // enable, autoset, reset to 0.0 
                    .reverseLimitType(ReverseLimitTypeValue.NormallyOpen)
                    .withVoltageLimits(8, -8)
                    .withStatorCurrentLimit(Amps.of(30.0))
                    .build();

            public static final class kStall {
                /**
                 * Stall detection tuning parameters for the Intake Arm.
                 *
                 * These values determine how the subsystem decides that the arm has
                 * mechanically stalled (i.e., the motors are applying effort but the
                 * mechanism is not moving). Adjusting these values changes how sensitive
                 * stall detection is.
                 *
                 * VELOCITY_THRESHOLD:
                 *   - Minimum mechanism velocity (in deg/sec) considered "moving".
                 *   - If the arm's measured velocity stays BELOW this value while effort
                 *     is being applied, it is considered "not moving".
                 *   - Increase this value → stall is detected MORE easily (more sensitive).
                 *   - Decrease this value → stall is detected LESS easily (less sensitive).
                 *
                 * EFFORT_THRESHOLD:
                 *   - Minimum applied motor voltage (in volts) considered "trying".
                 *   - If the motors are applying MORE than this voltage but velocity is low,
                 *     the subsystem considers the arm to be pushing against resistance.
                 *   - Increase this value → stall requires MORE effort to trigger (less sensitive).
                 *   - Decrease this value → stall triggers with LESS effort (more sensitive).
                 *
                 * TIME_SEC:
                 *   - Minimum duration (in seconds) that the stall condition must persist
                 *     before being considered a true stall.
                 *   - This prevents false positives from brief slowdowns, backlash, or noise.
                 *   - Increase this value → stall must last LONGER to trigger (less sensitive).
                 *   - Decrease this value → stall triggers FASTER (more sensitive).
                 *
                 * Together, these three values define the mechanical "signature" of a stall.
                 * Tune them based on real‑world behavior: heavier arms, higher friction, or
                 * slower gearboxes may require different thresholds.
                 */
                public static final double VELOCITY_THRESHOLD = 1.0;   // deg/sec
                public static final double EFFORT_THRESHOLD   = 2.0;   // volts
                public static final double TIME_SEC           = 0.10;  // seconds
            }

        }
    }

    // ╔═══════════════════════════════════════════════════════════════╗
    // ║                       SHOOTER CONSTANTS                       ║
    // ║               (Arcane Ballistic Projection Core)              ║
    // ╚═══════════════════════════════════════════════════════════════╝
    public static final class kShooter {

        // General shooter limits
        public static final class kRPM {
            public static final double MAX  = 4000.0;
            public static final double LOB  = 2650.0;
            public static final double IDLE = 500.0;
            public static final double SAFE = 2800.0;

            public static final double BUMP_RAYMOND = 2650;
        }

        public static final double TARGET_TOLERANCE = 20.0;
        

        // ------------------------------------------------------------
        // Motor tuning (PID + FF + voltage limits)
        // Hardware (CAN IDs, ratio, transform) is now in ShooterHardware
        // ------------------------------------------------------------

        public static final MotorConfig BLU_CONFIG =
            MotorConfig.builder(ControlMode.VELOCITY)
                .inverted(true)
                .withPID(0.1, 0.0, 0.02)
                .withFeedforward(0.32, 0.12231, 0.0)
                .withVoltageLimits(12.0, -12.0)
                .withStatorCurrentLimit(Amps.of(110))
                .build();


        public static final MotorConfig YEL_CONFIG =
            MotorConfig.builder(ControlMode.VELOCITY)
                .inverted(false)
                .withPID(0.1, 0.0, 0.02)
                .withFeedforward(0.32, 0.12231, 0.0)
                .withVoltageLimits(12.0, -12.0)
                .withStatorCurrentLimit(Amps.of(110))
                .build();


        // ------------------------------------------------------------
        // Simulation Behavior
        // ------------------------------------------------------------
        public static final class kSim {
            public static final double SENSOR_FILTER_TIME_CONSTANT = 0.04;
            public static final double COMMAND_FILTER_TIME_CONSTANT = 0.08;
            public static final double VOLTAGE_SLEW_RATE = 24.0;
            public static final double MAX_ACCEL_RPM_PER_SEC = 8000.0;
            public static final double MAX_VOLTAGE = 12.0;

            // Shooter flywheel physics model (shared by BLU and YEL)
            public static final FlywheelModel FLYWHEEL_MODEL =
                new FlywheelModel(
                    0.00032,                 // inertia (kg·m²)
                    55.7,                    // motor Kv (rad/s per volt)
                    0.018,                   // motor Kt (N·m per amp)
                    0.090,           // motor resistance (ohms)
                    0.01              // friction torque (N·m)
                );
        }
    }

    // ╔═══════════════════════════════════════════════════════════════╗
    // ║                       SWERVE CONSTANTS                        ║
    // ╚═══════════════════════════════════════════════════════════════╝
    public static final class kSwerve {

        public static final class kScale {
            public static final double CREEP    = 0.3;
            public static final double NORMAL   = 0.8;
        }
        
        public static final double MAX_SPEED =
            0.8 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

        public static final double MAX_ANGULAR_RATE =
            RotationsPerSecond.of(0.82100).in(RadiansPerSecond);
    }

    // ╔═══════════════════════════════════════════════════════════════╗
    // ║                   ROBOT GEOMETRY CONSTANTS                    ║
    // ║             (Spatial Frame of the Warlock Engine)             ║
    // ╚═══════════════════════════════════════════════════════════════╝
    public static final class kRobotGeometry {
        public static final double HALF_LENGTH_METERS = 0.35;
        public static final double HALF_WIDTH_METERS  = 0.35;
    }

    // ╔═══════════════════════════════════════════════════════════════╗
    // ║                 MOVE‑THROUGH‑POSE CONSTANTS                   ║
    // ║                (Pathfinding & Guidance Runes)                 ║
    // ╚═══════════════════════════════════════════════════════════════╝
    public static final class kMoveThroughPose {

        // PID gains for X/Y translation
        public static final double XY_KP = 0.4;
        public static final double XY_KI = 0.0;
        public static final double XY_KD = 0.0;

        // PID gains for rotation
        public static final double THETA_KP = 1.0;
        public static final double THETA_KI = 0.0;
        public static final double THETA_KD = 0.0;

        // Stall detection
        public static final double STALL_THRESHOLD = 0.02;
        public static final double STALL_TIMEOUT = 1.0;
    }

    // ╔═══════════════════════════════════════════════════════════════╗
    // ║                           QuestNav                            ║
    // ║                (Warlocks Arcane Optics Suite)                 ║
    // ╚═══════════════════════════════════════════════════════════════╝
    public static final class kQuest {

        public static final Matrix<N3, N1> STD_DEVS = VecBuilder.fill(0.02, 0.02, 0.05);

        public static final double ACCEPTABLE_DISTANCE_TOLERANCE = 0.1;
        
        // Mount transform: robot origin -> QuestNav sensor
        public static final Transform3d ROBOT_TO_QUEST =
            new Transform3d(
                -0.20226, 0.304165, 0.39,
                new Rotation3d(0.0, 0.0, Math.toRadians(90.0))
            );
    }
}
   
        
    