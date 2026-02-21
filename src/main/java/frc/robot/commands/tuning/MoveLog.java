//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.robot.commands.tuning;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Constants.kMoveToPose;

import java.io.*;
import java.text.SimpleDateFormat;
import java.util.*;

/**
 * MoveLog
 *
 * A fully self-contained MoveToPose tuning system.
 * Logs pose, target pose, commanded speeds, and actual speeds to CSV.
 * At the end of auto, analyzes all logs together and generates a text report.
 *
 * This file is designed to be completely removable before competition.
 */
public final class MoveLog {

    // ============================================================
    // Static global state
    // ============================================================

    private static MoveLog active = null;
    private static final List<File> completedLogs = new ArrayList<>();

    private static final SimpleDateFormat sdf = new SimpleDateFormat("yyyyMMdd_HHmmss");

    // ============================================================
    // Instance fields
    // ============================================================

    private final SwerveSubsystem swerve;
    private final Pose2d target;

    private File csvFile;
    private FileWriter writer;

    private double startTime;

    // ============================================================
    // Constructor
    // ============================================================

    private MoveLog(SwerveSubsystem swerve, Pose2d target) {
        this.swerve = swerve;
        this.target = target;
    }

    // ============================================================
    // Public API for AutoSequence
    // ============================================================

    public static Command startLog(SwerveSubsystem swerve, Pose2d target) {
        return Commands.runOnce(() -> {
            active = new MoveLog(swerve, target);
            active.start();
        });
    }

    public static Command record() {
        return Commands.run(() -> {
            if (active != null) {
                active.recordSample();
            }
        });
    }

    public static Command endLog() {
        return Commands.runOnce(() -> {
            if (active != null) {
                active.stop();
                completedLogs.add(active.csvFile);
                active = null;
            }
        });
    }

    public static Command analyzeAllLogs() {
        return Commands.runOnce(() -> {
            if (completedLogs.isEmpty()) {
                System.out.println("[MoveLog] No logs to analyze.");
                return;
            }
            Analyzer.run(completedLogs);
        });
    }

    // ============================================================
    // Logging lifecycle
    // ============================================================

    private void start() {
        try {
            String timestamp = sdf.format(new Date());
            File dir = new File(Filesystem.getOperatingDirectory(), "movelogs");
            dir.mkdirs();

            csvFile = new File(dir, "movelog_" + timestamp + ".csv");
            writer = new FileWriter(csvFile);

            writeHeader();
            startTime = Timer.getFPGATimestamp();

        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    private void stop() {
        try {
            if (writer != null) {
                writer.flush();
                writer.close();
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    // ============================================================
    // CSV Writing
    // ============================================================

    private void writeHeader() throws IOException {
        writer.write(
            "time," +
            "poseX,poseY,poseTheta," +
            "targetX,targetY,targetTheta," +
            "vx_cmd,vy_cmd,omega_cmd," +
            "vx_actual,vy_actual,omega_actual\n"
        );
    }

    private void recordSample() {
        try {
            double now = Timer.getFPGATimestamp();
            double t = now - startTime;

            var inputs = swerve.getInputs();

            Pose2d pose = inputs.pose;
            ChassisSpeeds actual = inputs.speeds;

            // Commanded speeds come from MoveToPose via MoveLog.record(commanded)
            ChassisSpeeds commanded = lastCommanded;

            if (commanded == null) {
                commanded = new ChassisSpeeds();
            }

            writer.write(
                t + "," +
                pose.getX() + "," + pose.getY() + "," + pose.getRotation().getRadians() + "," +
                target.getX() + "," + target.getY() + "," + target.getRotation().getRadians() + "," +
                commanded.vxMetersPerSecond + "," +
                commanded.vyMetersPerSecond + "," +
                commanded.omegaRadiansPerSecond + "," +
                actual.vxMetersPerSecond + "," +
                actual.vyMetersPerSecond + "," +
                actual.omegaRadiansPerSecond + "\n"
            );

        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    // ============================================================
    // Commanded speed hook (called from MoveToPose)
    // ============================================================

    private static ChassisSpeeds lastCommanded = null;

    public static void record(ChassisSpeeds commanded) {
        lastCommanded = commanded;
    }

    // ============================================================
    // Analyzer
    // ============================================================

    private static final class Analyzer {

        private static final class Sample {
            double time;

            double poseX, poseY, poseTheta;
            double targetX, targetY, targetTheta;

            double vx_cmd, vy_cmd, omega_cmd;
            double vx_actual, vy_actual, omega_actual;
        }

        private static final class Metrics {
            double maxOvershoot;
            double avgSlowdownDistance;
            double rotationLagScore;
            double oscillationScore;
            double minSpeedDeficiency;

            // New: jitter near target (small dithering instead of clean settle)
            double jitterScore;

            // New: rough settle time (time to first enter and stay within tolerance)
            double settleTime;
        }

        public static void run(List<File> logs) {
            try {
                List<Sample> all = new ArrayList<>();

                for (File f : logs) {
                    all.addAll(readCsv(f));
                }

                if (all.isEmpty()) {
                    System.out.println("[MoveLog] No samples found.");
                    return;
                }

                File report = createReportFile();
                FileWriter out = new FileWriter(report);

                out.write("=== MoveToPose Tuning Report ===\n\n");
                out.write("Total samples: " + all.size() + "\n");
                out.write("Logs analyzed: " + logs.size() + "\n\n");

                Metrics m = computeMetrics(all);

                out.write("=== Metrics ===\n");
                out.write("Max overshoot: " + m.maxOvershoot + " m\n");
                out.write("Avg slowdown distance: " + m.avgSlowdownDistance + " m\n");
                out.write("Rotation lag score: " + m.rotationLagScore + "\n");
                out.write("Oscillation score: " + m.oscillationScore + "\n");
                out.write("Jitter score (near target): " + m.jitterScore + "\n");
                out.write("Min-speed deficiency: " + m.minSpeedDeficiency + "\n");
                out.write("Approx settle time: " + m.settleTime + " s\n\n");

                double confidence = computeConfidence(m);
                out.write("Confidence Score: " + confidence + "\n\n");

                // Current constants from kMoveToPose
                double xyKp = kMoveToPose.XY_KP;
                double xyKi = kMoveToPose.XY_KI;
                double xyKd = kMoveToPose.XY_KD;

                double thetaKp = kMoveToPose.THETA_KP;
                double thetaKi = kMoveToPose.THETA_KI;
                double thetaKd = kMoveToPose.THETA_KD;

                double slowdownStart = kMoveToPose.SLOWDOWN_START;
                double slowdownMin = kMoveToPose.SLOWDOWN_MIN;
                double minSpeed = kMoveToPose.MIN_SPEED;

                double posTol = kMoveToPose.POSITION_TOLERANCE_METERS;
                double angTol = kMoveToPose.ANGLE_TOLERANCE_RADIANS;

                out.write("=== Current Constants ===\n");
                out.write("XY_KP = " + xyKp + "\n");
                out.write("XY_KI = " + xyKi + "\n");
                out.write("XY_KD = " + xyKd + "\n\n");

                out.write("THETA_KP = " + thetaKp + "\n");
                out.write("THETA_KI = " + thetaKi + "\n");
                out.write("THETA_KD = " + thetaKd + "\n\n");

                out.write("SLOWDOWN_START = " + slowdownStart + "\n");
                out.write("SLOWDOWN_MIN = " + slowdownMin + "\n");
                out.write("MIN_SPEED = " + minSpeed + "\n\n");

                out.write("POSITION_TOLERANCE = " + posTol + "\n");
                out.write("ANGLE_TOLERANCE = " + angTol + "\n\n");

                // Compute recommended constants
                Recommendations rec = computeRecommendations(m, xyKp, xyKd, thetaKd, slowdownStart, minSpeed);

                out.write("=== Recommended Constants ===\n");
                out.write("XY_KP: " + xyKp + " -> " + rec.xyKpNew +
                        "  (? " + (rec.xyKpNew - xyKp) + ")\n");
                out.write("XY_KD: " + xyKd + " -> " + rec.xyKdNew +
                        "  (? " + (rec.xyKdNew - xyKd) + ")\n");
                out.write("SLOWDOWN_START: " + slowdownStart + " -> " + rec.slowdownStartNew +
                        "  (? " + (rec.slowdownStartNew - slowdownStart) + ")\n");
                out.write("MIN_SPEED: " + minSpeed + " -> " + rec.minSpeedNew +
                        "  (? " + (rec.minSpeedNew - minSpeed) + ")\n");
                out.write("THETA_KD: " + thetaKd + " -> " + rec.thetaKdNew +
                        "  (? " + (rec.thetaKdNew - thetaKd) + ")\n\n");

                out.write("=== Explanation ===\n");
                out.write(generateSuggestions(m, rec));
                out.write("\n");

                out.flush();
                out.close();

                System.out.println("[MoveLog] Analysis complete. Report written to: " + report.getAbsolutePath());

            } catch (Exception e) {
                e.printStackTrace();
            }
        }

        // ------------------------------------------------------------
        // CSV Reader
        // ------------------------------------------------------------

        private static List<Sample> readCsv(File f) throws Exception {
            List<Sample> list = new ArrayList<>();

            BufferedReader br = new BufferedReader(new FileReader(f));
            String line = br.readLine(); // skip header

            while ((line = br.readLine()) != null) {
                String[] s = line.split(",");

                Sample sm = new Sample();
                sm.time = Double.parseDouble(s[0]);

                sm.poseX = Double.parseDouble(s[1]);
                sm.poseY = Double.parseDouble(s[2]);
                sm.poseTheta = Double.parseDouble(s[3]);

                sm.targetX = Double.parseDouble(s[4]);
                sm.targetY = Double.parseDouble(s[5]);
                sm.targetTheta = Double.parseDouble(s[6]);

                sm.vx_cmd = Double.parseDouble(s[7]);
                sm.vy_cmd = Double.parseDouble(s[8]);
                sm.omega_cmd = Double.parseDouble(s[9]);

                sm.vx_actual = Double.parseDouble(s[10]);
                sm.vy_actual = Double.parseDouble(s[11]);
                sm.omega_actual = Double.parseDouble(s[12]);

                list.add(sm);
            }

            br.close();
            return list;
        }

        // ------------------------------------------------------------
        // Metric computation (v3)
        // ------------------------------------------------------------

        private static Metrics computeMetrics(List<Sample> s) {
            Metrics m = new Metrics();

            double totalSlowdownDist = 0;
            int slowdownCount = 0;

            double maxOvershoot = 0;
            double rotationLag = 0;
            double oscillation = 0;
            double minSpeedDef = 0;
            double jitter = 0;

            // For settle time: first time we enter and stay within tolerance
            double posTol = kMoveToPose.POSITION_TOLERANCE_METERS;
            double angTol = kMoveToPose.ANGLE_TOLERANCE_RADIANS;
            boolean inTolerance = false;
            double firstInTolTime = -1.0;

            for (int i = 1; i < s.size(); i++) {
                Sample a = s.get(i - 1);
                Sample b = s.get(i);

                double dist = Math.hypot(b.poseX - b.targetX, b.poseY - b.targetY);
                double speedMag = Math.hypot(b.vx_actual, b.vy_actual);
                double cmdSpeedMag = Math.hypot(b.vx_cmd, b.vy_cmd);

                // Overshoot: near target but still moving fast
                if (dist < posTol && speedMag > 0.5) {
                    maxOvershoot = Math.max(maxOvershoot, speedMag);
                }

                // Slowdown distance: where commanded speed is small
                if (cmdSpeedMag < 0.5) {
                    totalSlowdownDist += dist;
                    slowdownCount++;
                }

                // Rotation lag: actual omega much smaller than commanded
                if (Math.abs(b.omega_cmd) > 0.1 &&
                    Math.abs(b.omega_actual) < Math.abs(b.omega_cmd) * 0.5) {
                    rotationLag += 1;
                }

                // Oscillation: sign changes of actual velocity at meaningful speed
                if (Math.signum(a.vx_actual) != Math.signum(b.vx_actual) &&
                    Math.abs(a.vx_actual) > 0.1 && Math.abs(b.vx_actual) > 0.1) {
                    oscillation += 1;
                }

                // Jitter near target: small dithering when we should be "done"
                if (dist < 2.0 * posTol &&
                    cmdSpeedMag < 0.3 &&
                    speedMag < 0.5 &&
                    Math.signum(a.vx_actual) != Math.signum(b.vx_actual)) {
                    jitter += 1;
                }

                // Min-speed deficiency: moving slower than half of MIN_SPEED
                if (speedMag < kMoveToPose.MIN_SPEED * 0.5 && dist > posTol) {
                    minSpeedDef += 1;
                }

                // Settle time: first time we enter and stay within tolerance
                boolean nowInTol = dist < posTol &&
                                Math.abs(wrapAngle(b.poseTheta - b.targetTheta)) < angTol;
                if (nowInTol && !inTolerance) {
                    firstInTolTime = b.time;
                    inTolerance = true;
                } else if (!nowInTol && inTolerance) {
                    // We left tolerance again; reset
                    inTolerance = false;
                    firstInTolTime = -1.0;
                }
            }

            m.maxOvershoot = maxOvershoot;
            m.avgSlowdownDistance = slowdownCount > 0 ? totalSlowdownDist / slowdownCount : 0;
            m.rotationLagScore = rotationLag;
            m.oscillationScore = oscillation;
            m.minSpeedDeficiency = minSpeedDef;
            m.jitterScore = jitter;
            m.settleTime = firstInTolTime < 0 ? s.get(s.size() - 1).time : firstInTolTime;

            return m;
        }

        private static double wrapAngle(double angle) {
            while (angle > Math.PI) angle -= 2.0 * Math.PI;
            while (angle < -Math.PI) angle += 2.0 * Math.PI;
            return angle;
        }

        // ------------------------------------------------------------
        // Confidence score (v3)
        // ------------------------------------------------------------

        private static double computeConfidence(Metrics m) {
            double score = 1.0;

            // Overshoot is bad but you currently have 0.0, so light weight
            score -= m.maxOvershoot * 0.3;

            // Oscillation and jitter are what make it feel "jittery"
            score -= m.oscillationScore * 0.015;
            score -= m.jitterScore * 0.03;

            // Min-speed deficiency matters, but less than jitter
            score -= m.minSpeedDeficiency * 0.005;

            // Very long settle time should hurt confidence
            if (m.settleTime > 1.5) {
                score -= (m.settleTime - 1.5) * 0.1;
            }

            return Math.max(0, Math.min(1, score));
        }

        // ------------------------------------------------------------
        // Recommendation structure
        // ------------------------------------------------------------

        private static final class Recommendations {
            double xyKpNew;
            double xyKdNew;
            double slowdownStartNew;
            double minSpeedNew;
            double thetaKdNew;
        }

        // ------------------------------------------------------------
        // Recommendation logic (v3)
        // ------------------------------------------------------------

        private static Recommendations computeRecommendations(
                Metrics m,
                double xyKp, double xyKd, double thetaKd,
                double slowdownStart, double minSpeed) {

            Recommendations r = new Recommendations();

            double newKp = xyKp;
            double newKd = xyKd;
            double newSlowdownStart = slowdownStart;
            double newMinSpeed = minSpeed;
            double newThetaKd = thetaKd;

            // Target slowdown distance ~ SLOWDOWN_START
            double desiredSlowdown = slowdownStart;
            if (m.avgSlowdownDistance > desiredSlowdown + 0.05) {
                newSlowdownStart -= 0.04;
            } else if (m.avgSlowdownDistance < desiredSlowdown - 0.05) {
                newSlowdownStart += 0.04;
            }

            // Min-speed deficiency: robot crawling far from target
            if (m.minSpeedDeficiency > 50) {
                newMinSpeed += 0.05;
            }

            // Oscillation + jitter logic:
            //  - First try increasing KD up to a ceiling
            //  - If KD is already high, start backing off KP
            if (m.oscillationScore > 40 || m.jitterScore > 20) {
                if (newKd < 0.35) {
                    newKd = Math.min(0.35, newKd + 0.05);
                } else {
                    // KD already high: reduce KP instead of cranking KD forever
                    newKp = Math.max(1.5, newKp * 0.85);
                }
            }

            // If overshoot ever appears, reduce KP a bit
            if (m.maxOvershoot > 0.1) {
                newKp = Math.max(1.0, newKp * 0.9);
            }

            // Theta KD: only if rotation lag is significant
            if (m.rotationLagScore > 10 && newThetaKd < 0.3) {
                newThetaKd = Math.min(0.3, newThetaKd + 0.05);
            }

            r.xyKpNew = newKp;
            r.xyKdNew = newKd;
            r.slowdownStartNew = newSlowdownStart;
            r.minSpeedNew = newMinSpeed;
            r.thetaKdNew = newThetaKd;

            return r;
        }

        // ------------------------------------------------------------
        // Suggestions text
        // ------------------------------------------------------------

        private static String generateSuggestions(Metrics m, Recommendations r) {
            StringBuilder sb = new StringBuilder();

            if (m.oscillationScore > 40 || m.jitterScore > 20) {
                sb.append("• High oscillation/jitter detected near target (osc=")
                .append(m.oscillationScore)
                .append(", jitter=")
                .append(m.jitterScore)
                .append("). Consider the recommended XY_KP/XY_KD changes.\n");
            }

            if (m.avgSlowdownDistance > kMoveToPose.SLOWDOWN_START + 0.05) {
                sb.append("• Slowdown appears to start too early (avg distance = ")
                .append(m.avgSlowdownDistance)
                .append(" m).\n");
            } else if (m.avgSlowdownDistance < kMoveToPose.SLOWDOWN_START - 0.05) {
                sb.append("• Slowdown appears to start too late (avg distance = ")
                .append(m.avgSlowdownDistance)
                .append(" m).\n");
            }

            if (m.minSpeedDeficiency > 50) {
                sb.append("• Robot spends a lot of time below half MIN_SPEED far from target (")
                .append(m.minSpeedDeficiency)
                .append(" samples). Increasing MIN_SPEED may help it commit to motion.\n");
            }

            if (m.maxOvershoot > 0.1) {
                sb.append("• Some overshoot detected near the target (max speed ")
                .append(m.maxOvershoot)
                .append(" m/s). Reducing XY_KP slightly can help.\n");
            }

            if (sb.length() == 0) {
                sb.append("No major issues detected. Motion looks reasonably tuned; fine-tune KP/KD based on feel if desired.\n");
            }

            return sb.toString();
        }

        // ------------------------------------------------------------
        // Report file creation
        // ------------------------------------------------------------

        private static File createReportFile() {
            try {
                File dir = new File(Filesystem.getOperatingDirectory(), "movelogs");
                dir.mkdirs();

                String timestamp = sdf.format(new Date());
                return new File(dir, "movelog_report_" + timestamp + ".txt");

            } catch (Exception e) {
                e.printStackTrace();
                return new File("movelog_report_fallback.txt");
            }
        }
    }
}
