package frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
    @AutoLog
    public static class VisionIOInputs {
        public boolean connected = false;
        public double[] tagDistances = new double[0];
        public long[] tagIDs = new long[0];
        public double minTagDistance = 0.0;
        public double estimatedPoseX = 0.0;
        public double estimatedPoseY = 0.0;
        public double estimatedPoseRotDeg = 0.0;
        public double timestampSeconds = 0.0;
        public boolean hasValidEstimate = false;
    }

    public default void updateInputs(VisionIOInputs inputs) {}
}
