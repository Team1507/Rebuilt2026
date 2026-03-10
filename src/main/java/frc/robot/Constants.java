//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;

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
                .voltageLimits(3.5, -3.5)
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
                .pid(0.11, 0.0, 0.0)
                .feedforward(0.245, 0.09375, 0.0)
                .gravity(0.0, GravityType.CONSTANT)
                .voltageLimits(12.0, -12.0)
                .build();

        public static final MotorConfig CONFIG_SLOT1 =
            MotorConfig.builder(ControlMode.MOTION_MAGIC)
                .slot(1)
                .pid(0.11, 0.0, 0.0)
                .feedforward(0.245, 0.09375, 0.0)
                .gravity(0.8, GravityType.CONSTANT)
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
                .pid(0.11, 0.0, 0.0)
                .feedforward(0.42, 0.09931, 0.0)
                .voltageLimits(12.0, -12.0)
                .build();

        public static final MotorConfig YEL_CONFIG =
            MotorConfig.builder(ControlMode.VELOCITY)
                .inverted(false)
                .pid(0.11, 0.0, 0.02)
                .feedforward(0.42, 0.09931, 0.0)
                .voltageLimits(12.0, -12.0)
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
                .pid(0.11, 0.0, 0.02)
                .voltageLimits(8.0, -8.0)
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
        
        

        public static final MotorConfig ROLLER_CONFIG =
            MotorConfig.builder(ControlMode.DUTY_CYCLE)
                .voltageLimits(7.0, -7.0)
                .build();

        public static final class kArm {

            public static final double MAX_ANGLE_DEGREES = 138.0;
            public static final double MIN_ANGLE_DEGREES = 0.0;
            public static final double DEPLOYED_ANGLE_DEGREES = 135.0;
            public static final double RETRACTED_ANGLE_DEGREES = 82.0;

            public static final double MANUAL_POSITIVE_POWER = 0.4;
            public static final double MANUAL_NEGATIVE_POWER = -0.4;

            public static final MotorConfig BLU_CONFIG =
                MotorConfig.builder(ControlMode.POSITION)
                    .inverted(false)
                    .pid(0.5, 0.0, 0.0)
                    .gravity(0.1, GravityType.COSINE)
                    .reverseLimit(true, true, 0.0) // enable, autoset, reset to 0.0 
                    .reverseLimitType(ReverseLimitTypeValue.NormallyOpen)
                    .voltageLimits(8, -8)
                    .withBrake()
                    .build();


            public static final MotorConfig YEL_CONFIG =
                MotorConfig.builder(ControlMode.POSITION)
                    .inverted(true)
                    .pid(0.5, 0.0, 0.0)
                    .gravity(0.1, GravityType.COSINE)
                    .reverseLimit(true, true, 0.0) // enable, autoset, reset to 0.0 
                    .reverseLimitType(ReverseLimitTypeValue.NormallyOpen)
                    .voltageLimits(8, -8)
                    .build();
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
                .pid(0.1, 0.0, 0.02)
                .feedforward(0.32, 0.12231, 0.0)
                .voltageLimits(12.0, -12.0)
                .build();


        public static final MotorConfig YEL_CONFIG =
            MotorConfig.builder(ControlMode.VELOCITY)
                .inverted(false)
                .pid(0.1, 0.0, 0.02)
                .feedforward(0.32, 0.12231, 0.0)
                .voltageLimits(12.0, -12.0)
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
    // ║                     MOVE‑TO‑POSE CONSTANTS                    ║
    // ║             (Pathfinding & Guidance Runes)             ║
    // ╚═══════════════════════════════════════════════════════════════╝
    public static final class kMoveToPose {

        // PID gains for X/Y translation
        public static final double XY_KP = 1.476; // increasing kP can decrease settle time
            /*Note: When a non‑zero settle time is observed, increasing XY_KP can reduce 
              terminal convergence time, provided overshoot, 
              oscillation, and jitter metrics remain clean. 
             */
        public static final double XY_KI = 0.0;
        public static final double XY_KD = 0.45;

        // PID gains for rotation
        public static final double THETA_KP = 1.4;
        public static final double THETA_KI = 0.0;
        public static final double THETA_KD = 0.0;

        // Deadband near target
        public static final double DEADBAND_ERROR = 0.05;
        public static final double SLOWDOWN_START = 0.6;
        public static final double SLOWDOWN_MIN = 0.2;

        // Minimum Speed for movements
        public static final double MIN_SPEED = 0.25;

        // Tolerances
        public static final double POSITION_TOLERANCE_METERS = 0.09;
        public static final double ANGLE_TOLERANCE_RADIANS = Math.toRadians(1.0);

        // Stall detection
        public static final double STALL_THRESHOLD = 0.05;  // m/s
        public static final double STALL_TIMEOUT = 0.35;    // seconds
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
                    new Translation3d(-0.201775, 0.246126, 0.69125),
                    new Rotation3d(0, Math.toRadians(10), Math.toRadians(3.5))
                );
        }

        /** Yellow-side camera configuration */
        public static final class YEL {
            public static final String NAME = "Yellowcam";
            public static final Transform3d ROBOT_TO_CAMERA =
                new Transform3d(
                    new Translation3d(-0.25, -0.25, 0.21),
                    new Rotation3d(0, Math.toRadians(10), Math.toRadians(234))
                );
        }
    }

}
   
        
    