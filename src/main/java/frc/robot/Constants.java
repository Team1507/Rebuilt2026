//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.*;
import frc.lib.util.MotorConfig;
import frc.lib.util.MotorConfig.ControlMode;
import frc.robot.generated.ctre.TunerConstants;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

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
        public static final MotorConfig CONFIG = new MotorConfig(
            // Output limits
            8, -8
        );

        /** Duty cycles for agitator behavior. */
        public static final double AGITATE_TO_SHOOTER_DUTY = 0.5;
        public static final double AGITATE_TO_INTAKE_DUTY = -0.5;
    }

    // ╔═══════════════════════════════════════════════════════════════╗
    // ║                       CLIMBER CONSTANTS                       ║
    // ║                (Ether‑Lift Ascension Mechanism)               ║
    // ╚═══════════════════════════════════════════════════════════════╝
    public static final class kClimber {

        /** Mechanism setpoints (in mechanism units, not motor rotations). */
        public static final double UP = 27.0;
        public static final double DOWN = 20.0;

        /** Robot-relative aliases for convenience. */
        public static final double ROBOT_UP = DOWN;
        public static final double ROBOT_DOWN = UP;

        /**
         * MotorConfig now contains ONLY tuning values.
         * Hardware (CAN ID, servo port, gear ratio) lives in ClimberHardware.
         */
        public static final MotorConfig CONFIG = new MotorConfig(
            ControlMode.MOTION_MAGIC,

            // PID slot 0
            0.11,   // kP
            0.0,    // kI
            0.0,    // kD

            // Motion Magic feedforward
            0.09375, // kS
            0.245,   // kV
            0.0,     // kA

            // Output limits
            8, -8
        );
    }

    // ╔═══════════════════════════════════════════════════════════════╗
    // ║                       FEEDER CONSTANTS                        ║
    // ║             (Mana‑Driven Material Uplift System)              ║
    // ╚═══════════════════════════════════════════════════════════════╝
    public static final class kFeeder {

        public static final double FEED_RPM = 500.0;
        public static final double VOMIT_RPM = -250.0;

        public static final MotorConfig CONFIG = new MotorConfig(
            // PID
            0.11, 0.0, 0.0,

            // Feedforward
            0.09375, 0.245, 0.0,

            // Voltage limits
            8, -8
        );
    }

    // ╔═══════════════════════════════════════════════════════════════╗
    // ║                       HOPPER CONSTANTS                        ║
    // ║                (Arcane Material Buffer Chamber)               ║
    // ╚═══════════════════════════════════════════════════════════════╝
    public static final class kHopper {

        /** Hopper target positions (in degrees). */ 
        public static final double LOAD_POS = 0.0; 
        public static final double SHOOT_POS = 90.0;

        /**
         * MotorConfig now contains ONLY tuning values.
         * Hardware (CAN ID, gear ratio) lives in HopperHardware.
         */
        public static final MotorConfig CONFIG = new MotorConfig(
            // PID
            0.11,  // kP
            0.0,   // kI
            0.0,   // kD

            // Feedforward (unused for position control)
            0.0,   // kS
            0.0,   // kV
            0.0,   // kA

            // Voltage limits
            8, -8
        );
    }

    // ╔═══════════════════════════════════════════════════════════════╗
    // ║                       INTAKE CONSTANTS                        ║
    // ║                 (Soul‑Siphon Acquisition Arm)                 ║
    // ╚═══════════════════════════════════════════════════════════════╝
    public static final class kIntake {

        public static final double INTAKE_ROLLER_DUTY = 0.5;
        public static final double OUTTAKE_ROLLER_DUTY = -0.5;

        public static final MotorConfig ROLLER_CONFIG = new MotorConfig(
            8, -8
        );

        public static final class kArm {

            public static final double MAX_ANGLE_DEGREES = 0.65;
            public static final double MIN_ANGLE_DEGREES = 0.0;
            public static final double DEPLOYED_ANGLE_DEGREES = 0.64;
            public static final double RETRACTED_ANGLE_DEGREES = 0.15;

            public static final MotorConfig BLU_CONFIG = new MotorConfig(
                0.01, 0.0, 0.0,   // PID
                4, -4             // voltage limits
            );

            public static final MotorConfig YEL_CONFIG = new MotorConfig(
                0.01, 0.0, 0.0,
                4, -4
            );
        }
    }

    // ╔═══════════════════════════════════════════════════════════════╗
    // ║                       SHOOTER CONSTANTS                       ║
    // ║               (Arcane Ballistic Projection Core)              ║
    // ╚═══════════════════════════════════════════════════════════════╝
    public static final class kShooter {

        // General shooter limits
        public static final double MAX_RPM = 2400.0;
        public static final double TARGET_TOLERANCE = 2.0;

        // ------------------------------------------------------------
        // Motor tuning (PID + FF + voltage limits)
        // Hardware (CAN IDs, ratio, transform) is now in ShooterHardware
        // ------------------------------------------------------------

        public static final MotorConfig BLU_CONFIG = new MotorConfig(
            /* kP */ 0.02,
            /* kI */ 0.0,
            /* kD */ 0.0,

            /* kV */ 0.1052631579,
            /* kS */ 0.245,
            /* kA */ 0.0,

            /* peakForwardVoltage */ 8,
            /* peakReverseVoltage */ -8
        );

        public static final MotorConfig YEL_CONFIG = new MotorConfig(
            /* kP */ 0.02,
            /* kI */ 0.0,
            /* kD */ 0.0,

            /* kV */ 0.1052631579,
            /* kS */ 0.245,
            /* kA */ 0.0,

            /* peakForwardVoltage */ 8,
            /* peakReverseVoltage */ -8
        );

        // ------------------------------------------------------------
        // Simulation Behavior
        // ------------------------------------------------------------
        public static final class kSim {
            public static final double SENSOR_FILTER_TIME_CONSTANT = 0.04;
            public static final double COMMAND_FILTER_TIME_CONSTANT = 0.08;
            public static final double VOLTAGE_SLEW_RATE = 24.0;
            public static final double MAX_ACCEL_RPM_PER_SEC = 8000.0;
            public static final double MAX_VOLTAGE = 12.0;
        }
    }

    // ╔═══════════════════════════════════════════════════════════════╗
    // ║                       SWERVE CONSTANTS                        ║
    // ╚═══════════════════════════════════════════════════════════════╝
    public static final class kSwerve {
        public static double MAX_SPEED =
            0.5 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

        public static double MAX_ANGULAR_RATE =
            RotationsPerSecond.of(0.75).in(RadiansPerSecond);
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
    // ║                     MOVE‑TO‑POSE CONSTANTS                    ║
    // ║             (Arcane Pathfinding & Guidance Runes)             ║
    // ╚═══════════════════════════════════════════════════════════════╝
    public static final class kMoveToPose {

        // PID gains for X/Y translation
        public static final double XY_KP = 0.3;
        public static final double XY_KI = 0.0;
        public static final double XY_KD = 0.0;

        // PID gains for rotation
        public static final double THETA_KP = 1.0;
        public static final double THETA_KI = 0.0;
        public static final double THETA_KD = 0.0;

        // Deadband near target
        public static final double DEADBAND_ERROR = 0.05;

        // Tolerances
        public static final double POSITION_TOLERANCE_METERS = 0.09;
        public static final double ANGLE_TOLERANCE_RADIANS = Math.toRadians(3.0);

        // Stall detection
        public static final double STALL_THRESHOLD = 0.02;
        public static final double STALL_TIMEOUT = 1.0;
    }

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
    // ║                       VISION CONSTANTS                        ║
    // ║                (Warlocks Arcane Optics Suite)                 ║
    // ╚═══════════════════════════════════════════════════════════════╝
    public static final class kVision {

        /** AprilTag field layout for the current game */
        public static AprilTagFieldLayout APRILTAG_LAYOUT =
            AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

        // Rejection thresholds
        public static double maxAmbiguity = 0.3; 
        public static double maxZError = 0.75;
        public static double maxTagDistance = 6.0; // meters — tags beyond this are rejected

        // Standard deviation bases (multiplied by average tag distance in Vision.java).
        // Multi-tag observations get a sqrt(tagCount) discount on top of this.
        // Trig solve is very accurate for XY (uses gyro heading directly).
        // Constrained PnP is less precise but also estimates heading.
        public static double trigXyStdBase = 0.3;
        public static double constrainedPnpXyStdBase = 0.6;
        public static double constrainedPnpAngStdBase = 0.25;

        // Per-camera trust multipliers (applied to std devs)
        public static double[] cameraStdDevFactors = {
            1.0, // Camera 0 (BLU)
            1.0  // Camera 1 (YEL)
        };

        /** Blue-side camera configuration */
        public static final class BLU {
            public static final String NAME = "Bluecam";
            public static final Transform3d ROBOT_TO_CAMERA =
                new Transform3d(
                    new Translation3d(-0.288925, 0.314325, 0.015875),
                    new Rotation3d(0, Math.toRadians(15), Math.toRadians(70))
                );
        }

        /** Yellow-side camera configuration */
        public static final class YEL {
            public static final String NAME = "Yellowcam";
            public static final Transform3d ROBOT_TO_CAMERA =
                new Transform3d(
                    new Translation3d(-0.288925, -0.314325, 0.015875),
                    new Rotation3d(0, Math.toRadians(15), Math.toRadians(210))
                );
        }
    }

    // ╔═══════════════════════════════════════════════════════════════╗
    // ║                            QuestNav                           ║
    // ╚═══════════════════════════════════════════════════════════════╝
    public static final class kQuest {

        public static final Matrix<N3, N1> STD_DEVS = VecBuilder.fill(0.02, 0.02, Double.POSITIVE_INFINITY);

        public static final double ACCEPTABLE_DISTANCE_TOLERANCE = 0.1;
        
        // Mount transform: robot origin -> QuestNav sensor
        public static final Transform3d ROBOT_TO_QUEST =
            new Transform3d(
                -0.381, 0.0, 0.307,
                new Rotation3d(0.0, 0.0, Math.toRadians(180.0))
            );
    }
}
   
        
    