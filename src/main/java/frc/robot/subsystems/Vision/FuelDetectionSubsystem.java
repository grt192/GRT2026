package frc.robot.subsystems.Vision;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Value;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Objects;
import java.util.Optional;
import java.util.function.Function;
import frc.robot.util.RollingAverage;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FuelDetectionSubsystem extends SubsystemBase {

    public record Detection(
        double timestampSeconds,
        double yawDegrees,
        double pitchDegrees,
        double area,
        Optional<Distance> distanceMeters) {
    }

    public static record FuelDetectionConfig(String cameraName, Optional<Distance> cameraHeight, Optional<Distance> targetHeight, Optional<Angle> cameraPitch, int pipelineIndex) {
        public FuelDetectionConfig {
            Objects.requireNonNull(cameraName, "cameraName is required");
            Objects.requireNonNull(cameraHeight, "cameraHeight cannot be null, use Optional.empty()");
            Objects.requireNonNull(targetHeight, "targetHeight cannot be null, use Optional.empty()");
            Objects.requireNonNull(cameraPitch, "cameraPitch cannot be null, use Optional.empty()");
        }

        public FuelDetectionConfig(String cameraName, int pipelineIndex) {
            this(cameraName, Optional.empty(), Optional.empty(), Optional.empty(), pipelineIndex);
        }

        public boolean canCalculateDistance() {
            return cameraHeight.isPresent() && targetHeight.isPresent() && cameraPitch.isPresent();
        }
    }

    private static final Detection EMPTY_DETECTION = new Detection(
        Double.NaN,
        Double.NaN,
        Double.NaN,
        Double.NaN,
        Optional.empty());

    private final PhotonCamera camera;
    private int pipelineIndex;
    private final Optional<Distance> cameraHeight;
    private final Optional<Distance> targetHeight;
    private final Optional<Angle> cameraPitch;
    private final boolean canCalculateDistance;

    private final String dashboardPrefix;
    private final Function<String, String> dashboardKey;

    private List<Detection> latestDetections = List.of();
    private Optional<Detection> bestDetection = Optional.empty();

    private final RollingAverage countAverage = new RollingAverage(VisionConstants.FUEL_SMOOTHING_WINDOW_SIZE);
    private final RollingAverage distanceAverage = new RollingAverage(VisionConstants.FUEL_SMOOTHING_WINDOW_SIZE);
    private final RollingAverage minDistanceAverage = new RollingAverage(VisionConstants.FUEL_SMOOTHING_WINDOW_SIZE);
    private final RollingAverage maxDistanceAverage = new RollingAverage(VisionConstants.FUEL_SMOOTHING_WINDOW_SIZE);

    private Optional<Distance> filteredDistance = Optional.empty();
    private Optional<Distance> filteredMinDistance = Optional.empty();
    private Optional<Distance> filteredMaxDistance = Optional.empty();
    private Optional<Distance> filteredCount = Optional.empty();

    private Optional<Time> latestTimestamp = Optional.empty();
    private Optional<Time> startDecayTime = Optional.empty();

    public FuelDetectionSubsystem(FuelDetectionConfig config) {
        Objects.requireNonNull(config, "FuelDetectionConfig cannot be null");

        camera = new PhotonCamera(config.cameraName());
        pipelineIndex = config.pipelineIndex();
        cameraHeight = config.cameraHeight();
        targetHeight = config.targetHeight();
        cameraPitch = config.cameraPitch();
        canCalculateDistance = config.canCalculateDistance();
        camera.setPipelineIndex(pipelineIndex);

        dashboardPrefix = "FuelDetection/" + config.cameraName() + "/";
        dashboardKey = (suffix) -> (dashboardPrefix + suffix);
    }

    public void setPipelineIndex(int newPipelineIndex) {
        if (newPipelineIndex == pipelineIndex) {
            return;
        }
        pipelineIndex = newPipelineIndex;
        camera.setPipelineIndex(pipelineIndex);
    }

    public int getPipelineIndex() {
        return pipelineIndex;
    }

    public List<Detection> getDetections() {
        return List.copyOf(latestDetections);
    }

    public Optional<Detection> getBestDetection() {
        return bestDetection;
    }

    public boolean isFuelDetected() {
        return getBestDistance().isPresent();
    }

    public int getFuelCount() {
        return countAverage.getRoundedAverage();
    }

    private void handleResult(PhotonPipelineResult result, Time robotTimestamp) {
        List<Detection> processed = new ArrayList<>();

        if (result.hasTargets()) {
            for (PhotonTrackedTarget target : result.getTargets()) {
                processed.add(createDetection(result.getTimestampSeconds(), target));
            }
        }

        latestDetections = List.copyOf(processed);
        countAverage.addSample(processed.size());
        if (processed.isEmpty()) {
            bestDetection = Optional.empty();
            applyDecayToFilteredValues(robotTimestamp);
        } else {
            bestDetection = processed.stream()
                .max(Comparator.comparingDouble(Detection::area));
            Optional<Distance> minDistance = findDistanceExtreme(processed, true);
            Optional<Distance> maxDistance = findDistanceExtreme(processed, false);
            bestDetection.ifPresentOrElse(
                detection -> {
                    recordDetectionForSmoothing(detection, minDistance, maxDistance, robotTimestamp);
                },
                () -> applyDecayToFilteredValues(robotTimestamp));
        }
    }

    private Detection createDetection(double timestampSeconds, PhotonTrackedTarget target) {
        Optional<Distance> distanceMeters = Optional.empty();
        if (canCalculateDistance) {
            double calculatedMeters = PhotonUtils.calculateDistanceToTargetMeters(
                cameraHeight.get().in(Meters),
                targetHeight.get().in(Meters),
                cameraPitch.get().in(Radians),
                Units.degreesToRadians(target.getPitch()));
            if (Double.isFinite(calculatedMeters)) {
                distanceMeters = Optional.of(Meters.of(calculatedMeters));
            }
        }
        return new Detection(
            timestampSeconds,
            target.getYaw(),
            target.getPitch(),
            target.getArea(),
            distanceMeters);
    }

    private void recordDetectionForSmoothing(
        Detection detection,
        Optional<Distance> minDistance,
        Optional<Distance> maxDistance,
        Time robotTimestamp) {
        detection.distanceMeters().ifPresent(d -> {
            distanceAverage.addSample(d.in(Meters));
            filteredDistance = Optional.of(Meters.of(distanceAverage.getAverage()));
        });
        minDistance.ifPresent(d -> {
            minDistanceAverage.addSample(d.in(Meters));
            filteredMinDistance = Optional.of(Meters.of(minDistanceAverage.getAverage()));
        });
        maxDistance.ifPresent(d -> {
            maxDistanceAverage.addSample(d.in(Meters));
            filteredMaxDistance = Optional.of(Meters.of(maxDistanceAverage.getAverage()));
        });
        latestTimestamp = Optional.of(Seconds.of(detection.timestampSeconds()));
        startDecayTime = Optional.of(robotTimestamp.plus(VisionConstants.FUEL_DECAY_HOLD_TIME_SECONDS));
    }

    private void applyDecayToFilteredValues(Time timeNow) {
        if (startDecayTime.isEmpty()) {
            resetFilteredState();
            return;
        }
        Time decayStartTime = startDecayTime.get();
        if (timeNow.lte(decayStartTime)) {
            return;
        }

        Time elapsed = timeNow.minus(decayStartTime);
        double decayProgress = Math.min(elapsed.div(VisionConstants.FUEL_DECAY_TIME_SECONDS).in(Value), 1.0);
        double scale = Math.max(0.0, 1.0 - decayProgress);

        filteredDistance = filteredDistance.map(
            distance -> Meters.of(distance.in(Meters) * scale));
        filteredMinDistance = filteredMinDistance.map(
            distance -> Meters.of(distance.in(Meters) * scale));
        filteredMaxDistance = filteredMaxDistance.map(
            distance -> Meters.of(distance.in(Meters) * scale));

        if (decayProgress >= 1.0) {
            resetFilteredState();
            startDecayTime = Optional.empty();
        }
    }

    private void resetFilteredState() {
        filteredDistance = Optional.empty();
        filteredMinDistance = Optional.empty();
        filteredMaxDistance = Optional.empty();
        latestTimestamp = Optional.empty();
        countAverage.clear();
        distanceAverage.clear();
        minDistanceAverage.clear();
        maxDistanceAverage.clear();
    }

    private Optional<Distance> findDistanceExtreme(List<Detection> detections, boolean findMin) {
        Distance bestDistance = null;
        for (Detection detection : detections) {
            Optional<Distance> distance = detection.distanceMeters();
            if (distance.isEmpty()) {
                continue;
            }
            Distance candidate = distance.get();
            if (bestDistance == null) {
                bestDistance = candidate;
                continue;
            }
            double candidateMeters = candidate.in(Meters);
            double bestMeters = bestDistance.in(Meters);
            if (findMin ? candidateMeters < bestMeters : candidateMeters > bestMeters) {
                bestDistance = candidate;
            }
        }
        return Optional.ofNullable(bestDistance);
    }

    public Optional<Distance> getClosestDistance() {
        return filteredMinDistance;
    }

    public Optional<Distance> getFurthestDistance() {
        return filteredMaxDistance;
    }

    public Optional<Distance> getBestDistance() {
        return filteredDistance;
    }

    private void publishTelemetry() {
        SmartDashboard.putNumber(dashboardKey.apply("count"), latestDetections.size());

        double[] yawSamples = latestDetections.stream().mapToDouble(Detection::yawDegrees).toArray();
        double[] pitchSamples = latestDetections.stream().mapToDouble(Detection::pitchDegrees).toArray();

        SmartDashboard.putNumberArray(dashboardKey.apply("yawSamples"), yawSamples);
        SmartDashboard.putNumberArray(dashboardKey.apply("pitchSamples"), pitchSamples);

        double[] distanceSamples = latestDetections.stream()
            .map(Detection::distanceMeters)
            .filter(Optional::isPresent)
            .map(Optional::get)
            .mapToDouble(distance -> distance.in(Meters))
            .toArray();
        SmartDashboard.putNumberArray(dashboardKey.apply("distanceSamples"), distanceSamples);

        SmartDashboard.putNumber(dashboardKey.apply("bestYawDeg"), bestDetection.orElse(EMPTY_DETECTION).yawDegrees());
        SmartDashboard.putNumber(dashboardKey.apply("bestPitchDeg"), bestDetection.orElse(EMPTY_DETECTION).pitchDegrees());
        SmartDashboard.putNumber(dashboardKey.apply("bestDistanceMeters"), filteredDistance.orElse(Meters.of(Double.NaN)).in(Meters));
        SmartDashboard.putNumber(dashboardKey.apply("minDistanceMeters"), filteredMinDistance.orElse(Meters.of(Double.NaN)).in(Meters));
        SmartDashboard.putNumber(dashboardKey.apply("maxDistanceMeters"), filteredMaxDistance.orElse(Meters.of(Double.NaN)).in(Meters));
        SmartDashboard.putNumber(dashboardKey.apply("timestampSeconds"), latestTimestamp.orElse(Seconds.of(Double.NaN)).in(Seconds));
    }

    // Basically run handleResult() on pipeline results or run applyDecayToFilteredValues
    @Override
    public void periodic() {
        List<PhotonPipelineResult> results = camera.getAllUnreadResults();
        Time timeNow = Seconds.of(Timer.getFPGATimestamp());

        if (results.isEmpty()) {
            applyDecayToFilteredValues(timeNow);
            publishTelemetry();
            return;
        }

        for (PhotonPipelineResult result : results) {
            handleResult(result, timeNow);
        }
        publishTelemetry();
    }
}
