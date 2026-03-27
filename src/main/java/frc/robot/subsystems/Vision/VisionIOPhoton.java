package frc.robot.subsystems.Vision;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Filesystem;

public class VisionIOPhoton implements VisionIO {
    private final PhotonCamera camera;
    private final PhotonPoseEstimator photonPoseEstimator;
    private static AprilTagFieldLayout aprilTagFieldLayout;

    public VisionIOPhoton(CameraConfig cameraConfig) {
        camera = new PhotonCamera(cameraConfig.getCameraName());
        try {
            aprilTagFieldLayout = new AprilTagFieldLayout(
                Filesystem.getDeployDirectory() + "/2026-rebuilt-welded.json");
        } catch (Exception e) {
            throw new RuntimeException("Failed to load field layout", e);
        }
        photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, cameraConfig.getCameraPose());
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        inputs.connected = camera.isConnected();
        inputs.hasValidEstimate = false;

        List<PhotonPipelineResult> results = camera.getAllUnreadResults();
        for (PhotonPipelineResult result : results) {
            if (!result.hasTargets())
                continue;

            double minDistance = Double.MAX_VALUE;
            long[] tagIDs = new long[result.getTargets().size()];
            double[] tagDistances = new double[result.getTargets().size()];

            for (int i = 0; i < result.getTargets().size(); i++) {
                PhotonTrackedTarget target = result.getTargets().get(i);
                Translation3d translation = target.getBestCameraToTarget().getTranslation();
                double distance = Math.sqrt(
                    Math.pow(translation.getX(), 2) +
                        Math.pow(translation.getY(), 2) +
                        Math.pow(translation.getZ(), 2));
                if (distance < minDistance)
                    minDistance = distance;
                tagIDs[i] = target.getFiducialId();
                tagDistances[i] = distance;
            }

            inputs.tagDistances = tagDistances;
            inputs.tagIDs = tagIDs;
            inputs.minTagDistance = minDistance;

            if (minDistance > 4)
                continue;

            Optional<EstimatedRobotPose> estimatedPose = photonPoseEstimator.estimateAverageBestTargetsPose(result);
            if (!estimatedPose.isPresent())
                continue;

            Pose2d pose2d = estimatedPose.get().estimatedPose.toPose2d();
            inputs.estimatedPoseX = pose2d.getX();
            inputs.estimatedPoseY = pose2d.getY();
            inputs.estimatedPoseRotDeg = pose2d.getRotation().getDegrees();
            inputs.timestampSeconds = result.getTimestampSeconds();
            inputs.hasValidEstimate = true;
        }
    }
}
