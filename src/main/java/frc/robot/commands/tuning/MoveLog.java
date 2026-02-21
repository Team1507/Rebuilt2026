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

        // ------------------------------------------------------------
        // Sample + Metrics
        // ------------------------------------------------------------

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
            double jitterScore;
            double settleTime;

            // Extra internal structure for better reasoning
            double approachOscillation;
            double nearTargetOscillation;
            double avgCmdSpeedFar;
            double avgActualSpeedFar;
            double avgCmdSpeedNear;
            double avgActualSpeedNear;
        }

        private static final class Recommendations {
            double xyKpNew;
            double xyKdNew;
            double slowdownStartNew;
            double slowdownMinNew;
            double minSpeedNew;
            double thetaKdNew;
        }

        // ------------------------------------------------------------
        // Entry point
        // ------------------------------------------------------------

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
                
                if (Double.isNaN(m.settleTime)) {
                    out.write("Settle time: N/A (direct arrival)\n\n");
                } else {
                    out.write("Approx settle time: " + m.settleTime + " s\n\n");
                }

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
                Recommendations rec = computeRecommendations(
                    m,
                    xyKp, xyKd, thetaKd,
                    slowdownStart, slowdownMin, minSpeed
                );

                out.write("=== Recommended Constants ===\n");
                out.write("XY_KP: " + xyKp + " -> " + rec.xyKpNew +
                        "  (delta " + (rec.xyKpNew - xyKp) + ")\n");
                out.write("XY_KD: " + xyKd + " -> " + rec.xyKdNew +
                        "  (delta " + (rec.xyKdNew - xyKd) + ")\n");
                out.write("SLOWDOWN_START: " + slowdownStart + " -> " + rec.slowdownStartNew +
                        "  (delta " + (rec.slowdownStartNew - slowdownStart) + ")\n");
                out.write("SLOWDOWN_MIN: " + slowdownMin + " -> " + rec.slowdownMinNew +
                        "  (delta " + (rec.slowdownMinNew - slowdownMin) + ")\n");
                out.write("MIN_SPEED: " + minSpeed + " -> " + rec.minSpeedNew +
                        "  (delta " + (rec.minSpeedNew - minSpeed) + ")\n");
                out.write("THETA_KD: " + thetaKd + " -> " + rec.thetaKdNew +
                        "  (delta" + (rec.thetaKdNew - thetaKd) + ")\n\n");

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
        // Metric computation (v4)
        // ------------------------------------------------------------

        private static Metrics computeMetrics(List<Sample> s) {
            Metrics m = new Metrics();

            double totalSlowdownDist = 0.0;
            int slowdownCount = 0;

            double maxOvershoot = 0.0;
            double rotationLag = 0.0;
            double oscillation = 0.0;
            double minSpeedDef = 0.0;
            double jitter = 0.0;

            double approachOsc = 0.0;
            double nearOsc = 0.0;

            double sumCmdFar = 0.0, sumActFar = 0.0;
            int countFar = 0;
            double sumCmdNear = 0.0, sumActNear = 0.0;
            int countNear = 0;

            double posTol = kMoveToPose.POSITION_TOLERANCE_METERS;
            double angTol = kMoveToPose.ANGLE_TOLERANCE_RADIANS;

            double settleStartTime = Double.NaN;
            double settleEndTime = Double.NaN;

            for (int i = 1; i < s.size(); i++) {
                Sample a = s.get(i - 1);
                Sample b = s.get(i);

                double dx = b.poseX - b.targetX;
                double dy = b.poseY - b.targetY;
                double dist = Math.hypot(dx, dy);

                double vxA = a.vx_actual;
                double vyA = a.vy_actual;
                double vxB = b.vx_actual;
                double vyB = b.vy_actual;

                double speedMag = Math.hypot(vxB, vyB);
                double cmdSpeedMag = Math.hypot(b.vx_cmd, b.vy_cmd);

                boolean far = dist > 3.0 * posTol;
                boolean near = dist < 2.0 * posTol;

                // Overshoot: moving away from target while inside tolerance
                if (dist < posTol) {
                    double velDot = vxB * dx + vyB * dy;
                    if (velDot > 0.0 && speedMag > 0.2) {
                        maxOvershoot = Math.max(maxOvershoot, speedMag);
                    }
                }

                // Slowdown distance
                if (cmdSpeedMag < 0.5 && dist > posTol) {
                    totalSlowdownDist += dist;
                    slowdownCount++;
                }

                // Rotation lag
                if (Math.abs(b.omega_cmd) > 0.1 &&
                    Math.abs(b.omega_actual) < Math.abs(b.omega_cmd) * 0.5) {
                    rotationLag += 1.0;
                }

                // Vector oscillation
                double dotPrev = vxA * vxB + vyA * vyB;
                double magPrev = Math.hypot(vxA, vyA);
                double magCurr = Math.hypot(vxB, vyB);

                boolean meaningfulSpeed = magPrev > 0.1 && magCurr > 0.1;
                boolean directionFlip = dotPrev < 0.0;

                if (meaningfulSpeed && directionFlip) {
                    oscillation += 1.0;
                    if (far) approachOsc += 1.0;
                    else if (near) nearOsc += 1.0;
                }

                // Jitter near target
                if (near && cmdSpeedMag < 0.3 && speedMag < 0.5 && directionFlip) {
                    jitter += 1.0;
                }

                // Min-speed deficiency
                if (speedMag < kMoveToPose.MIN_SPEED * 0.5 && dist > 2.0 * posTol) {
                    minSpeedDef += 1.0;
                }

                // Speed statistics
                if (far) {
                    sumCmdFar += cmdSpeedMag;
                    sumActFar += speedMag;
                    countFar++;
                } else if (near) {
                    sumCmdNear += cmdSpeedMag;
                    sumActNear += speedMag;
                    countNear++;
                }

                // -----------------------------
                // Settle time (MIN_SPEED → target)
                // -----------------------------

                boolean atTarget =
                    dist < posTol &&
                    Math.abs(wrapAngle(b.poseTheta - b.targetTheta)) < angTol;

                if (Double.isNaN(settleStartTime) &&
                    cmdSpeedMag <= kMoveToPose.MIN_SPEED + 1e-3) {
                    settleStartTime = b.time;
                }

                if (!Double.isNaN(settleStartTime) && atTarget) {
                    settleEndTime = b.time;
                    break;
                }
            }

            if (!Double.isNaN(settleStartTime) && !Double.isNaN(settleEndTime)) {
                m.settleTime = settleEndTime - settleStartTime;
            } else {
                m.settleTime = Double.NaN;
            }

            m.maxOvershoot = maxOvershoot;
            m.avgSlowdownDistance = slowdownCount > 0
                ? totalSlowdownDist / slowdownCount
                : 0.0;

            m.rotationLagScore = rotationLag;
            m.oscillationScore = oscillation;
            m.minSpeedDeficiency = minSpeedDef;
            m.jitterScore = jitter;

            m.approachOscillation = approachOsc;
            m.nearTargetOscillation = nearOsc;

            m.avgCmdSpeedFar = countFar > 0 ? sumCmdFar / countFar : 0.0;
            m.avgActualSpeedFar = countFar > 0 ? sumActFar / countFar : 0.0;
            m.avgCmdSpeedNear = countNear > 0 ? sumCmdNear / countNear : 0.0;
            m.avgActualSpeedNear = countNear > 0 ? sumActNear / countNear : 0.0;

            return m;
        }

        private static double wrapAngle(double angle) {
            while (angle > Math.PI) angle -= 2.0 * Math.PI;
            while (angle < -Math.PI) angle += 2.0 * Math.PI;
            return angle;
        }

        // ------------------------------------------------------------
        // Confidence score (v4)
        // ------------------------------------------------------------

        private static double computeConfidence(Metrics m) {
            double score = 1.0;

            // Overshoot: bad but rare in vector control
            score -= m.maxOvershoot * 0.4;

            // Oscillation and jitter: primary "feel" killers
            score -= m.approachOscillation * 0.01;
            score -= m.nearTargetOscillation * 0.02;
            score -= m.jitterScore * 0.03;

            // Min-speed deficiency: robot unwilling to move
            score -= m.minSpeedDeficiency * 0.005;

            // Very long settle time hurts confidence
            if (m.settleTime > 1.5) {
                score -= (m.settleTime - 1.5) * 0.1;
            }

            return Math.max(0.0, Math.min(1.0, score));
        }

        // ------------------------------------------------------------
        // Recommendation logic (v4)
        // ------------------------------------------------------------

        private static Recommendations computeRecommendations(
                Metrics m,
                double xyKp, double xyKd, double thetaKd,
                double slowdownStart, double slowdownMin, double minSpeed) {

            Recommendations r = new Recommendations();

            double newKp = xyKp;
            double newKd = xyKd;
            double newSlowdownStart = slowdownStart;
            double newSlowdownMin = slowdownMin;
            double newMinSpeed = minSpeed;
            double newThetaKd = thetaKd;

            // 1. SlowdownStart: only if motion is otherwise clean
            boolean motionClean =
                m.jitterScore < 10 &&
                m.nearTargetOscillation < 10 &&
                m.maxOvershoot < 0.1;

            boolean slowdownCurveActive =
                kMoveToPose.SLOWDOWN_START > 0.3 &&
                m.avgCmdSpeedFar > 0.6;

            if (motionClean && slowdownCurveActive) {
                double desiredSlowdown = slowdownStart;
                if (m.avgSlowdownDistance > desiredSlowdown + 0.05) {
                    // slowing too early -> start slowdown later
                    newSlowdownStart = slowdownStart - 0.04;
                } else if (m.avgSlowdownDistance < desiredSlowdown - 0.05) {
                    // slowing too late -> start slowdown earlier
                    newSlowdownStart = slowdownStart + 0.04;
                }
                newSlowdownStart = Math.max(newSlowdownStart, 0.2);
            }

            // 2. MIN_SPEED: two-sided logic
            // Far from target but crawling -> increase MIN_SPEED
            if (m.minSpeedDeficiency > 50 && m.avgCmdSpeedFar > 0.4) {
                newMinSpeed = minSpeed + 0.05;
            }

            // Near target jitter + early slowdown -> MIN_SPEED likely too high
            if (m.jitterScore > 20 &&
                m.avgSlowdownDistance < slowdownStart - 0.1) {
                newMinSpeed = Math.max(0.15, minSpeed - 0.05);
            }

            // 3. SLOWDOWN_MIN tuning
            // Too much jitter near target → SLOWDOWN_MIN too high
            if (m.jitterScore > 20 &&
                m.avgCmdSpeedNear > 0.3 &&
                m.nearTargetOscillation > 10) {
                newSlowdownMin = Math.max(0.05, slowdownMin - 0.02);
            }

            // Robot dies early → SLOWDOWN_MIN too low
            if (m.avgActualSpeedNear < 0.2 &&
                m.avgCmdSpeedNear < 0.2 &&
                m.nearTargetOscillation < 5 &&
                m.jitterScore < 10 &&
                m.settleTime > 1.0) {
                newSlowdownMin = Math.min(0.4, slowdownMin + 0.02);
            }

            // 4. XY PID: oscillation / jitter tuning
            double kdMax = 0.45;
            double kpMin = 1.0;

            if (m.nearTargetOscillation > 20 || m.jitterScore > 20) {
                // First, try increasing KD up to kdMax
                if (newKd < kdMax) {
                    newKd = Math.min(kdMax, newKd + 0.05);
                } else {
                    // KD already high: reduce KP
                    newKp = Math.max(kpMin, newKp * 0.9);
                }
            }

            // Approach oscillation: robot "snakes" on the way in
            if (m.approachOscillation > 30 && newKd < kdMax) {
                newKd = Math.min(kdMax, newKd + 0.05);
            }

            // Overshoot: reduce KP a bit
            if (m.maxOvershoot > 0.1) {
                newKp = Math.max(kpMin, newKp * 0.9);
            }

            // 5. Theta KD: rotation lag
            if (m.rotationLagScore > 10 && newThetaKd < 0.3) {
                newThetaKd = Math.min(0.3, newThetaKd + 0.05);
            }

            r.xyKpNew = newKp;
            r.xyKdNew = newKd;
            r.slowdownStartNew = newSlowdownStart;
            r.slowdownMinNew = newSlowdownMin;
            r.minSpeedNew = newMinSpeed;
            r.thetaKdNew = newThetaKd;

            return r;
        }

        // ------------------------------------------------------------
        // Suggestions text
        // ------------------------------------------------------------

        private static String generateSuggestions(Metrics m, Recommendations r) {
            StringBuilder sb = new StringBuilder();

            if (m.nearTargetOscillation > 20 || m.jitterScore > 20) {
                sb.append("- Significant jitter/oscillation near the target (nearOsc=")
                .append(m.nearTargetOscillation)
                .append(", jitter=")
                .append(m.jitterScore)
                .append("). The recommended XY_KP/XY_KD and MIN_SPEED/SLOWDOWN_MIN changes aim to improve final settle behavior.\n");
            }

            if (m.approachOscillation > 30) {
                sb.append("- Noticeable oscillation while approaching the target (approachOsc=")
                .append(m.approachOscillation)
                .append("). Increasing XY_KD slightly can help damp the approach.\n");
            }

            // Slowdown / decel interpretation
            boolean slowdownTuned =
                Math.abs(r.slowdownStartNew - kMoveToPose.SLOWDOWN_START) > 1e-9;

            // If PID dominates, say that—and do NOT also talk about changing slowdown start.
            if (!slowdownTuned) {
                sb.append("- Deceleration is dominated by PID behavior rather than the slowdown curve. ")
                .append("Further SLOWDOWN_START tuning is unlikely to improve performance. ")
                .append("(avg distance = ")
                .append(m.avgSlowdownDistance)
                .append(" m)\n");
            } else {
                if (m.avgSlowdownDistance > kMoveToPose.SLOWDOWN_START + 0.05) {
                    sb.append("- Slowdown appears to start too early (avg distance = ")
                    .append(m.avgSlowdownDistance)
                    .append(" m). The suggested SLOWDOWN_START change will delay deceleration.\n");
                } else if (m.avgSlowdownDistance < kMoveToPose.SLOWDOWN_START - 0.05) {
                    sb.append("- Slowdown appears to start too late (avg distance = ")
                    .append(m.avgSlowdownDistance)
                    .append(" m). The suggested SLOWDOWN_START change will start deceleration sooner.\n");
                }
            }

            if (m.minSpeedDeficiency > 50) {
                sb.append("- Robot spends a lot of time below half MIN_SPEED far from target (")
                .append(m.minSpeedDeficiency)
                .append(" samples). Increasing MIN_SPEED may help it commit to motion.\n");
            }

            if (m.maxOvershoot > 0.1) {
                sb.append("- Some overshoot detected near the target (max speed ")
                .append(m.maxOvershoot)
                .append(" m/s). Reducing XY_KP slightly can help.\n");
            }

            if (m.rotationLagScore > 10) {
                sb.append("- Rotation lags behind commanded omega (lag samples=")
                .append(m.rotationLagScore)
                .append("). Increasing THETA_KD can help the robot track heading more crisply.\n");
            }

            if (sb.length() == 0) {
                sb.append("No major issues detected. Motion looks reasonably tuned; fine-tune KP/KD, SLOWDOWN_START, and SLOWDOWN_MIN based on feel if desired.\n");
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
