//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.robot.localization.vision;

import java.util.Optional;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonPipelineResult;

import frc.lib.math.geometry.VisionMeasurement;
import frc.lib.util.Alliance;

public final class PVPerCameraProcessor {

    public enum TagMode {
        ALL_TAGS,
        HUB,
        TOWER
    }

    /** Debug packet for PVManager → PVDebugInfo */
    public static final class ProcessorDebug {
        public String strategyUsed = "NONE";
        public boolean accepted = false;
        public String rejectionReason = "";

        public int tagCount = 0;
        public double avgDistance = 0.0;
        public double ambiguity = 0.0;

        public double xyStd = 0.0;
        public double angStd = 0.0;
    }

    /** Final usable measurement for PVManager fusion */
    public static final class VisionResult {
        public final VisionMeasurement measurement;
        public final int tagCount;
        public final double avgDistance;
        public final double ambiguity;

        public VisionResult(
            VisionMeasurement measurement,
            int tagCount,
            double avgDistance,
            double ambiguity
        ) {
            this.measurement = measurement;
            this.tagCount = tagCount;
            this.avgDistance = avgDistance;
            this.ambiguity = ambiguity;
        }
    }

    private final PhotonPoseEstimator estimator;
    private final String name;

    // Strategy weights (match 340 defaults)
    private double multiTagXy = 0.1;
    private double multiTagAng = 0.25;

    private double trigXy = 0.06;
    private double trigAng = 1e5;

    private double ambXy = 0.2;
    private double ambAng = 0.4;

    private double zTolerance = 0.6;

    private TagMode tagMode = TagMode.HUB;

    public PVPerCameraProcessor(String name, PhotonPoseEstimator estimator) {
        this.name = name;
        this.estimator = estimator;
    }

    public void setTagMode(TagMode mode) {
        this.tagMode = mode;
    }

    // ----------------------------------------------------------------------
    // DEBUG PATH (no VisionMeasurement, only debug info)
    // ----------------------------------------------------------------------
    public Optional<ProcessorDebug> processDebug(PhotonPipelineResult result, boolean enabled) {
        ProcessorDebug dbg = new ProcessorDebug();

        if (result == null || !result.hasTargets()) {
            dbg.rejectionReason = "NO_TARGETS";
            return Optional.of(dbg);
        }

        Optional<EstimatedRobotPose> estimate = Optional.empty();
        double xyWeight = 1e5;
        double angWeight = 1e5;

        // -------------------------
        // Stage 1: MultiTag
        // -------------------------
        var multi = result.getMultiTagResult();
        if (multi.isPresent()
            && !multi.get().fiducialIDsUsed.isEmpty()
            && useTag(multi.get().fiducialIDsUsed.get(0))) {

            estimate = estimator.estimateCoprocMultiTagPose(result);
            if (estimate.isPresent()) {
                dbg.strategyUsed = "MULTITAG";
                xyWeight = multiTagXy;
                angWeight = multiTagAng;
            }
        }

        // -------------------------
        // Stage 2: Trig-solve
        // -------------------------
        if (estimate.isEmpty() && enabled) {
            int id = result.getBestTarget().fiducialId;
            if (useTag(id)) {
                var trig = estimator.estimatePnpDistanceTrigSolvePose(result);
                if (trig.isPresent()) {
                    estimate = trig;
                    dbg.strategyUsed = "TRIG";
                    xyWeight = trigXy;
                    angWeight = trigAng;
                }
            }
        }

        // -------------------------
        // Stage 3: Lowest ambiguity
        // -------------------------
        if (estimate.isEmpty() && !enabled) {
            var amb = estimator.estimateLowestAmbiguityPose(result);
            if (amb.isPresent()) {
                boolean invalid = false;
                for (var t : amb.get().targetsUsed) {
                    if (!useTag(t.fiducialId)) invalid = true;
                }
                if (!invalid) {
                    estimate = amb;
                    dbg.strategyUsed = "AMBIGUITY";
                    xyWeight = ambXy;
                    angWeight = ambAng;
                }
            }
        }

        if (estimate.isEmpty()) {
            dbg.rejectionReason = "NO_STRATEGY_SUCCEEDED";
            return Optional.of(dbg);
        }

        EstimatedRobotPose est = estimate.get();
        Pose3d pose3d = est.estimatedPose;

        // Z-height rejection
        if (Math.abs(pose3d.getZ()) > zTolerance) {
            dbg.rejectionReason = "Z_OUT_OF_RANGE";
            return Optional.of(dbg);
        }

        // Compute metrics
        dbg.tagCount = est.targetsUsed.size();
        dbg.avgDistance = est.targetsUsed.stream()
            .mapToDouble(t -> t.getBestCameraToTarget().getTranslation().getNorm())
            .average().orElse(0.0);

        dbg.ambiguity = est.targetsUsed.stream()
            .mapToDouble(t -> t.poseAmbiguity)
            .average().orElse(0.0);

        dbg.xyStd = xyWeight * dbg.avgDistance * dbg.avgDistance;
        dbg.angStd = angWeight * dbg.avgDistance * dbg.avgDistance;

        dbg.accepted = true;
        dbg.rejectionReason = "";

        return Optional.of(dbg);
    }

    // ----------------------------------------------------------------------
    // MEASUREMENT PATH (used by PVManager for fusion)
    // ----------------------------------------------------------------------
    public Optional<VisionResult> process(PhotonPipelineResult result, boolean enabled) {
        if (result == null || !result.hasTargets()) return Optional.empty();

        Optional<EstimatedRobotPose> estimate = Optional.empty();
        double xyWeight = 1e5;
        double angWeight = 1e5;

        // Stage 1: MultiTag
        var multi = result.getMultiTagResult();
        if (multi.isPresent()
            && !multi.get().fiducialIDsUsed.isEmpty()
            && useTag(multi.get().fiducialIDsUsed.get(0))) {

            estimate = estimator.estimateCoprocMultiTagPose(result);
            if (estimate.isPresent()) {
                xyWeight = multiTagXy;
                angWeight = multiTagAng;
            }
        }

        // Stage 2: Trig-solve
        if (estimate.isEmpty() && enabled) {
            int id = result.getBestTarget().fiducialId;
            if (useTag(id)) {
                estimate = estimator.estimatePnpDistanceTrigSolvePose(result);
                if (estimate.isPresent()) {
                    xyWeight = trigXy;
                    angWeight = trigAng;
                }
            }
        }

        // Stage 3: Lowest ambiguity
        if (estimate.isEmpty() && !enabled) {
            estimate = estimator.estimateLowestAmbiguityPose(result);
            if (estimate.isPresent()) {
                boolean invalid = false;
                for (var t : estimate.get().targetsUsed) {
                    if (!useTag(t.fiducialId)) invalid = true;
                }
                if (invalid) {
                    estimate = Optional.empty();
                } else {
                    xyWeight = ambXy;
                    angWeight = ambAng;
                }
            }
        }

        if (estimate.isEmpty()) return Optional.empty();

        EstimatedRobotPose est = estimate.get();
        Pose3d pose3d = est.estimatedPose;

        if (Math.abs(pose3d.getZ()) > zTolerance) return Optional.empty();

        double avgDistance = est.targetsUsed.stream()
            .mapToDouble(t -> t.getBestCameraToTarget().getTranslation().getNorm())
            .average().orElse(0.0);

        double ambiguity = est.targetsUsed.stream()
            .mapToDouble(t -> t.poseAmbiguity)
            .average().orElse(0.0);

        int tagCount = est.targetsUsed.size();

        double xyStd = xyWeight * avgDistance * avgDistance;
        double angStd = angWeight * avgDistance * avgDistance;

        VisionMeasurement meas = new VisionMeasurement(
            pose3d.toPose2d(),
            est.timestampSeconds,
            VecBuilder.fill(xyStd, xyStd, angStd)
        );

        return Optional.of(new VisionResult(meas, tagCount, avgDistance, ambiguity));
    }

    // ----------------------------------------------------------------------
    // Tag filtering
    // ----------------------------------------------------------------------
    private boolean useTag(int id) {
        boolean invalidAlliance = DriverStation.getAlliance().isEmpty();

        switch (tagMode) {
            case ALL_TAGS:
                return id >= 1 && id <= 32;

            case HUB:
                boolean blueHub = (id >= 18 && id <= 21) || (id >= 24 && id <= 27);
                boolean redHub = (id >= 2 && id <= 5) || (id >= 8 && id <= 11);
                return invalidAlliance ? (blueHub || redHub)
                    : (Alliance.isBlue() ? blueHub : redHub);

            case TOWER:
                boolean blueTower = id == 31 || id == 32;
                boolean redTower = id == 15 || id == 16;
                return invalidAlliance ? (blueTower || redTower)
                    : (Alliance.isBlue() ? blueTower : redTower);
        }
        return false;
    }
}
