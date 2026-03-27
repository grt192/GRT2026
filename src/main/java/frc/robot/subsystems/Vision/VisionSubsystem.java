package frc.robot.subsystems.Vision;

import java.util.function.Consumer;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.PolynomialRegression;
import org.littletonrobotics.junction.Logger;

public class VisionSubsystem extends SubsystemBase {
    private final VisionIO io;
    private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();
    private final String name;

    private Consumer<TimestampedVisionUpdate> visionConsumer = (x) -> {
    };

    private PolynomialRegression xStdDevModel = VisionConstants.xStdDevModel;
    private PolynomialRegression yStdDevModel = VisionConstants.yStdDevModel;
    private PolynomialRegression oStdDevModel = VisionConstants.oStdDevModel;

    private Transform3d latestTransform3d = new Transform3d();

    public VisionSubsystem(String name, VisionIO io) {
        this.name = name;
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Vision/" + name, inputs);

        if (inputs.hasValidEstimate) {
            Pose2d estimatedPose2d = new Pose2d(
                inputs.estimatedPoseX,
                inputs.estimatedPoseY,
                Rotation2d.fromDegrees(inputs.estimatedPoseRotDeg));

            visionConsumer.accept(
                new TimestampedVisionUpdate(
                    inputs.timestampSeconds,
                    estimatedPose2d,
                    VecBuilder.fill(
                        xStdDevModel.predict(inputs.minTagDistance),
                        yStdDevModel.predict(inputs.minTagDistance),
                        oStdDevModel.predict(inputs.minTagDistance))));

            Logger.recordOutput("Vision/" + name + "/EstimatedPose", estimatedPose2d);
        }
    }

    public void setInterface(Consumer<TimestampedVisionUpdate> consumer) {
        visionConsumer = consumer;
    }

    public Transform3d cameraToApriltag() {
        return latestTransform3d;
    }

    public String getCamID() {
        return name;
    }
}
