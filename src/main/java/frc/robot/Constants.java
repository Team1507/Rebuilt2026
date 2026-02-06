//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;

/**
 * Central location for all robot-wide constants.
 * 
 * Subsystems should reference values from here rather than hardcoding
 * numbers in multiple places. This improves readability, tuning, and
 * maintainability across the project.
 */
public class Constants {

    // ╔═══════════════════════════════════════════════════════════════╗
    // ║                       VISION CONSTANTS                        ║
    // ║                (Warlocks Arcane Optics Suite)                 ║
    // ╚═══════════════════════════════════════════════════════════════╝
    public static final class kVision {

        /** AprilTag field layout for the current game */
        public static AprilTagFieldLayout APRILTAG_LAYOUT =
            AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

        // Basic filtering thresholds
        public static double maxAmbiguity = 0.3;
        public static double maxZError = 0.75;

        // Baseline standard deviations (scaled by distance and tag count)
        public static double linearStdDevBaseline = 0.02;   // meters
        public static double angularStdDevBaseline = 0.06;  // radians

        // Per-camera trust multipliers
        public static double[] cameraStdDevFactors = {
            1.0, // Camera 0
            1.0  // Camera 1
        };

        // MegaTag2 multipliers
        public static double linearStdDevMegatag2Factor = 0.5;
        public static double angularStdDevMegatag2Factor = Double.POSITIVE_INFINITY;

        /** Blue-side camera configuration */
        public static final class BLU {
            public static final String NAME = "Bluecam";
            public static final Transform3d ROBOT_TO_CAMERA =
                new Transform3d(
                    new Translation3d(-0.281, 0.296, 0.2413),
                    new Rotation3d(0, Math.toRadians(10), Math.toRadians(60))
                );
        }

        /** Yellow-side camera configuration */
        public static final class YEL {
            public static final String NAME = "Yellowcam";
            public static final Transform3d ROBOT_TO_CAMERA =
                new Transform3d(
                    new Translation3d(-0.2985, -0.276, 0.2413),
                    new Rotation3d(0, Math.toRadians(10), Math.toRadians(205))
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

        /** Blue shooter configuration */
        public static final class BLU {
            public static final int CAN_ID = 19;

            // Shooter position relative to robot center
            public static final Transform2d ROBOT_TO_SHOOTER =
                new Transform2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0));

            /** PID + feedforward gains */
            public static final class kGains {
                public static final double KP = 0.02;
                public static final double KI = 0.0;
                public static final double KD = 0.0;

                public static final double KV = 0.1052631579;
                public static final double KS = 0.2;
                public static final double KA = 0.0;
            }
        }

        /** Yellow shooter configuration */
        public static final class YEL {
            public static final int CAN_ID = 17;

            public static final Transform2d ROBOT_TO_SHOOTER =
                new Transform2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0));

            public static final class kGains {
                public static final double KP = 0.02;
                public static final double KI = 0.0;
                public static final double KD = 0.0;

                public static final double KV = 0.1052631579;
                public static final double KS = 0.2;
                public static final double KA = 0.0;
            }
        }

        // ---------------- Simulation Behavior ----------------
        public static final class kSim {
            public static final double SENSOR_FILTER_TIME_CONSTANT = 0.04;
            public static final double COMMAND_FILTER_TIME_CONSTANT = 0.08;
            public static final double VOLTAGE_SLEW_RATE = 24.0;
            public static final double MAX_ACCEL_RPM_PER_SEC = 8000.0;
            public static final double MAX_VOLTAGE = 12.0;
        }
    }

    // ╔═══════════════════════════════════════════════════════════════╗
    // ║                       FEEDER CONSTANTS                        ║
    // ║             (Mana‑Driven Material Uplift System)              ║
    // ╚═══════════════════════════════════════════════════════════════╝
    public static final class kFeeder {

        /** Blue feeder motor */
        public static final class BLU {
            public static final int CAN_ID = 20;

            public static final class kGains {
                public static final double KP = 0.11;
                public static final double KI = 0.0;
                public static final double KD = 0.0;

                public static final double KV = 0.09375;
                public static final double KS = 0.245;
                public static final double KA = 0.0;
            }
        }

        /** Yellow feeder motor */
        public static final class YEL {
            public static final int CAN_ID = 18;

            public static final class kGains {
                public static final double KP = 0.11;
                public static final double KI = 0.0;
                public static final double KD = 0.0;

                public static final double KV = 0.09375;
                public static final double KS = 0.245;
                public static final double KA = 0.0;
            }
        }
    }

    // ╔═══════════════════════════════════════════════════════════════╗
    // ║                       INTAKE CONSTANTS                        ║
    // ║                 (Soul‑Siphon Acquisition Arm)                 ║
    // ╚═══════════════════════════════════════════════════════════════╝
    public static final class kIntake {

        /** Roller motor */
        public static final class kRoller {
            public static final int CAN_ID = 13;

            public static final class kGains {
                public static final double KP = 0.013;
                public static final double KI = 0.0;
                public static final double KD = 0.0;

                public static final double KV = 0.10333857939;
                public static final double KS = 0.025;
                public static final double KA = 0.0;
            }
        }

        /** Intake arm motors + limits */
        public static final class kArm {

            // Arm angle limits
            public static final double MAX_ANGLE_DEGREES = 90.0;
            public static final double MIN_ANGLE_DEGREES = 0.0;
            public static final double DEPLOYED_ANGLE_DEGREES = 75.0;
            public static final double RETRACTED_ANGLE_DEGREES = 0.0;

            /** Blue arm motor */
            public static final class BLU {
                public static final int CAN_ID = 14;

                public static final class kGains {
                    public static final double KP = 1.0;
                    public static final double KI = 0.0;
                    public static final double KD = 0.0;

                    public static final double KV = 0.0929007;
                    public static final double KS = 0.018;
                    public static final double KA = 0.0;
                }
            }

            /** Yellow arm motor */
            public static final class YEL {
                public static final int CAN_ID = 27;

                public static final class kGains {
                    public static final double KP = 1.0;
                    public static final double KI = 0.0;
                    public static final double KD = 0.0;

                    public static final double KV = 0.0929007;
                    public static final double KS = 0.018;
                    public static final double KA = 0.0;
                }
            }
        }
    }

    // ╔═══════════════════════════════════════════════════════════════╗
    // ║                      AGITATOR CONSTANTS                       ║
    // ║               (Arcane Material Stirring Engine)               ║
    // ╚═══════════════════════════════════════════════════════════════╝
    public static final class kAgitator {
        public static final int CAN_ID = 15;

        public static final class kGains {
            public static final double KP = 0.11;
            public static final double KI = 0.0;
            public static final double KD = 0.0;

            public static final double KV = 0.09375;
            public static final double KS = 0.245;
            public static final double KA = 0.0;
        }
    }

    // ╔═══════════════════════════════════════════════════════════════╗
    // ║                       CLIMBER CONSTANTS                       ║
    // ║                (Ether‑Lift Ascension Mechanism)               ║
    // ╚═══════════════════════════════════════════════════════════════╝
    public static final class kClimber {
        public static final int CAN_ID = 23;

        public static final class kGains {
            public static final double KP = 0.11;
            public static final double KI = 0.0;
            public static final double KD = 0.0;

            public static final double KV = 0.09375;
            public static final double KS = 0.245;
            public static final double KA = 0.0;
        }
    }

    // ╔═══════════════════════════════════════════════════════════════╗
    // ║                       HOPPER CONSTANTS                        ║
    // ║                (Arcane Material Buffer Chamber)               ║
    // ╚═══════════════════════════════════════════════════════════════╝
    public static final class kHopper {
        public static final int CAN_ID = 23;

        public static final class kGains {
            public static final double KP = 0.11;
            public static final double KI = 0.0;
            public static final double KD = 0.0;

            public static final double KV = 0.09375;
            public static final double KS = 0.245;
            public static final double KA = 0.0;
        }
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
        public static final double X_KP = 1.0;
        public static final double X_KI = 0.0;
        public static final double X_KD = 0.0;

        public static final double Y_KP = 1.0;
        public static final double Y_KI = 0.0;
        public static final double Y_KD = 0.0;

        // PID gains for rotation
        public static final double THETA_KP = 2.0;
        public static final double THETA_KI = 0.0;
        public static final double THETA_KD = 0.0;

        // Deadband near target
        public static final double DEADBAND_ERROR = 0.05;

        // Tolerances
        public static final double POSITION_TOLERANCE_METERS = 0.05;
        public static final double ANGLE_TOLERANCE_RADIANS = Math.toRadians(2.0);

        // Stall detection
        public static final double STALL_THRESHOLD = 0.02;
        public static final double STALL_TIMEOUT = 1.0;
    }
}
