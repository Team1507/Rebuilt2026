//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.robot.localization;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.core.logging.Telemetry;
import frc.robot.localization.vision.PVManager;
import frc.robot.localization.vision.QuestNavManager;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * LocalizationManager:
 *
 *  - QuestNav is primary during ENABLED (translation + heading)
 *  - PhotonVision only corrects when avgDistance < 2m and stable
 *  - PhotonVision fully controls pose when DISABLED
 *  - Gyro is fallback only
 *  - Startup seeding uses PV stability
 */
public class LocalizationManager extends SubsystemBase {

    private final SwerveSubsystem swerve;
    private final PVManager pv;
    private final QuestNavManager quest;
    private final Telemetry telemetry;

    private int pvStableCount = 0;

    // 20 Hz throttle
    private double lastProcessTime = 0.0;
    private static final double PERIOD = 0.05;

    public LocalizationManager(
        SwerveSubsystem swerve,
        PVManager pv,
        QuestNavManager quest,
        Telemetry telemetry
    ) {
        this.swerve = swerve;
        this.pv = pv;
        this.quest = quest;
        this.telemetry = telemetry;

        telemetry.registerVisionPoseSource("Localization-Fused");
    }

    private boolean pvClose() {
        return pv.hasGoodVision() && pv.getFusedAvgDistance() < 2.0;
    }

    @Override
    public void periodic() {

        double now = Timer.getFPGATimestamp();
        if (now - lastProcessTime < PERIOD) return;
        lastProcessTime = now;

        boolean enabled = DriverStation.isEnabled();

        // ============================
        // 1. Read all sources
        // ============================

        var pvPoseOpt = pv.getFusedPose();
        boolean pvGood = pv.hasGoodVision();
        boolean pvClose = pvClose();

        boolean questGood = quest.isTracking();
        Pose2d questPose = quest.getPose2d();
        double questTs = quest.getTimestamp();

        Pose2d odomPose = swerve.getPose();

        // ============================
        // 2. Disabled set pose based on PhotonVision
        // PV must be stable AND QuestNav must be tracking
        // ============================

        if (!enabled) {

            boolean pvStable =
                pvGood &&
                pv.getFusedXyStd() < 1.0 &&
                pv.getFusedAngStd() < 1.0;

            if (pvStable) pvStableCount++;
            else pvStableCount = 0;

            if (pvStableCount >= 5 && questGood) {
                Pose2d seed = pvPoseOpt.get();

                // Seed drivetrain
                swerve.setPoseFromVision(seed);

                // Seed QuestNav
                quest.setPose(new Pose3d(seed));
            }
        }

        // ============================
        // 3. Choose translation source
        // ============================

        Pose2d translationSource;

        if (!enabled) {
            // DISABLED → PhotonVision ONLY
            if (pvGood) {
                translationSource = pvPoseOpt.get();
                telemetry.logLocalizationTranslationSource("PhotonVision (Disabled)");
            } else {
                translationSource = odomPose;
                telemetry.logLocalizationTranslationSource("Odometry (Disabled-Fallback)");
            }
        }
        else {
            // ENABLED → QuestNav ONLY
            if (questGood) {
                translationSource = questPose;
                telemetry.logLocalizationTranslationSource("QuestNav");
            } else {
                translationSource = odomPose;
                telemetry.logLocalizationTranslationSource("Odometry (Fallback)");
            }
        }

        // ============================
        // 4. Choose heading source
        // ============================

        Rotation2d heading;

        if (!enabled) {
            // DISABLED → PhotonVision ONLY
            if (pvGood) {
                heading = pvPoseOpt.get().getRotation();
                telemetry.logLocalizationHeadingSource("PhotonVision (Disabled)");
            } else {
                heading = swerve.getHeading();
                telemetry.logLocalizationHeadingSource("Gyro (Disabled-Fallback)");
            }
        }
        else {
            // ENABLED → QuestNav ONLY
            if (questGood) {
                heading = questPose.getRotation();
                telemetry.logLocalizationHeadingSource("QuestNav");
            } else {
                heading = swerve.getHeading();
                telemetry.logLocalizationHeadingSource("Gyro (Fallback)");
            }
        }

        // ============================
        // 5. Build fused pose
        // ============================

        Pose2d fusedPose = new Pose2d(
            translationSource.getX(),
            translationSource.getY(),
            heading
        );

        telemetry.logLocalizationFusedPose(fusedPose);

        // ============================
        // 6. Feed fused pose into drivetrain
        // ============================

        if (!enabled) {
            // DISABLED → PhotonVision ONLY sets pose
            if (pvGood) {
                swerve.setPoseFromVision(pvPoseOpt.get());
            }
            return;
        }

        // ENABLED → QuestNav ONLY contributes
        if (questGood) {
            swerve.addVisionMeasurement(
                questPose,
                questTs,
                frc.robot.Constants.kQuest.STD_DEVS
            );
        }
    }

    // ============================
    // Public API
    // ============================

    public Pose2d getFusedPose() {
        return swerve.getPose();
    }

    public void resetQuestPose(Pose2d pose) {
        quest.setPose(new Pose3d(pose));
    }

    public void resetPose(Pose2d resetPose) {
        // Reset the QuestPose
        resetQuestPose(resetPose);
        // Reset the drivetrain pose estimator to this new pose
        swerve.setPoseFromVision(resetPose);
    }

    public void resetHeadingToZero() {
        // 1. Get the current fused pose (this is what the robot "thinks" it is)
        Pose2d current = getFusedPose();

        // 2. Build a new pose with the same X/Y but heading = 0
        Pose2d zeroed = new Pose2d(
            current.getX(),
            current.getY(),
            new Rotation2d(0.0)
        );

        // 3. Reset the drivetrain pose estimator to this new pose
        swerve.setPoseFromVision(zeroed);
    }
}
