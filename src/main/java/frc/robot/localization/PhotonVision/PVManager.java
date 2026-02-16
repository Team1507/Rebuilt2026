//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.robot.localization.PhotonVision;

import static frc.robot.Constants.kVision.*;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.LinkedList;
import java.util.List;

import frc.lib.io.photonvision.PhotonVisionIO;
import frc.lib.io.photonvision.PhotonVisionIO.CameraInputs;
import frc.lib.logging.Telemetry;
import frc.robot.Constants.kQuest;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * IO‑based PhotonVision manager.
 *
 * This class no longer touches PhotonCamera or PhotonPoseEstimator directly.
 * All camera data comes from PhotonVisionIO, and all pose fusion is done
 * through SwerveSubsystem.
 */
public class PVManager extends SubsystemBase {

    private final Telemetry telemetry;
    private final PhotonVisionIO io;
    private final SwerveSubsystem swerve;
    private final java.util.function.Consumer<Pose3d> questNavPoseSeeder;

    private final CameraState[] cameras;

    private boolean startupPoseSeeded = false;

    // 20 Hz throttle
    private double lastProcessTime = 0.0;
    private static final double PROCESS_PERIOD = 0.05;

    public PVManager(
        Telemetry telemetry,
        PhotonVisionIO io,
        SwerveSubsystem swerve,
        java.util.function.Consumer<Pose3d> questNavPoseSeeder
    ) {
        this.telemetry = telemetry;
        this.io = io;
        this.swerve = swerve;
        this.questNavPoseSeeder = questNavPoseSeeder;

        cameras = new CameraState[io.getCameraCount()];
        for (int i = 0; i < cameras.length; i++) {
            cameras[i] = new CameraState("Camera-" + i);
            telemetry.registerVisionPoseSource("Camera-" + i);
        }
    }

    /** Internal per-camera state (debounce, alerts, etc.) */
    private static class CameraState {
        final String name;
        final Debouncer disabledDebounce =
            new Debouncer(5.0, DebounceType.kFalling);
        final Alert disconnectedAlert;

        Rotation2d latestYaw = Rotation2d.kZero;
        Rotation2d latestPitch = Rotation2d.kZero;
        boolean hasTarget = false;

        CameraState(String name) {
            this.name = name;
            this.disconnectedAlert = new Alert(
                "Vision camera " + name + " is disconnected.",
                AlertType.kWarning);
        }
    }

    public Rotation2d getTargetX(int index) {
        return cameras[index].latestYaw;
    }

    public Rotation2d getTargetY(int index) {
        return cameras[index].latestPitch;
    }

    public boolean hasTarget(int index) {
        return cameras[index].hasTarget;
    }

    @Override
    public void periodic() {

        // 20 Hz throttle
        double now = Timer.getFPGATimestamp();
        if (now - lastProcessTime < PROCESS_PERIOD) return;
        lastProcessTime = now;

        // Pull fresh camera data from IO
        CameraInputs[] inputs = new CameraInputs[cameras.length];
        for (int i = 0; i < inputs.length; i++) inputs[i] = new CameraInputs();
        io.updateInputs(inputs);

        List<Pose3d> accepted = new LinkedList<>();
        List<Pose3d> rejected = new LinkedList<>();
        List<Pose3d> tagPoses = new LinkedList<>();

        Pose2d seedPose = null;
        double seedScore = Double.NEGATIVE_INFINITY;
        int fusedCount = 0;

        Rotation2d heading = swerve.getHeading();

        for (int i = 0; i < cameras.length; i++) {
            CameraState cam = cameras[i];
            CameraInputs in = inputs[i];

            cam.hasTarget = in.hasTarget;
            cam.latestYaw = in.latestYaw;
            cam.latestPitch = in.latestPitch;

            if (in.latestPose3d.isEmpty()) continue;

            Pose3d pose3d = in.latestPose3d.get();
            Pose2d pose2d = in.latestPose2d.get();

            int tagCount = in.tagCount;
            double avgDistance = in.avgDistance;

            boolean reject =
                Math.abs(pose3d.getZ()) > maxZError ||
                pose3d.getX() < 0.0 ||
                pose3d.getX() > APRILTAG_LAYOUT.getFieldLength() ||
                pose3d.getY() < 0.0 ||
                pose3d.getY() > APRILTAG_LAYOUT.getFieldWidth() ||
                avgDistance > maxTagDistance ||
                (tagCount == 1 && in.stdDevs != null &&
                 in.stdDevs.get(2, 0) > maxAmbiguity);

            if (reject) {
                rejected.add(pose3d);
                continue;
            }

            accepted.add(pose3d);

            // Startup seeding logic
            if (!startupPoseSeeded) {
                double score = tagCount * 100.0 - avgDistance * 10.0;
                if (score > seedScore) {
                    seedScore = score;
                    seedPose = pose2d;
                }
            } else {
                // Vision fusion
                if (in.stdDevs != null) {
                    swerve.addVisionMeasurement(pose2d, in.timestampSeconds, in.stdDevs);
                    fusedCount++;
                }
            }

            telemetry.logVisionPose(cam.name, pose2d);
        }

        // Perform startup pose seed
        if (!startupPoseSeeded && seedPose != null) {
            swerve.seedPoseFromVision(seedPose);
            questNavPoseSeeder.accept(new Pose3d(seedPose).transformBy(kQuest.ROBOT_TO_QUEST));
            startupPoseSeeded = true;
            telemetry.logVisionStartupSeed(seedPose);
        }

        telemetry.logVisionAccepted(accepted.toArray(new Pose3d[0]));
        telemetry.logVisionRejected(rejected.toArray(new Pose3d[0]));
        telemetry.logVisionTagPoses(tagPoses.toArray(new Pose3d[0]));
        telemetry.logVisionFusedCount(fusedCount);
    }

    public boolean isVisionSeeded() {
        return startupPoseSeeded;
    }
}
