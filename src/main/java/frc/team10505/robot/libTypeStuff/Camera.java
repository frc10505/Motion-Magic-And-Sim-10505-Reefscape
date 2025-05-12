package frc.team10505.robot.libTypeStuff;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.targeting.PhotonPipelineResult;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;

public class Camera {
    private final PhotonCamera camera;
    private final PhotonPoseEstimator poseEstimator;
    private PhotonCameraSim cameraSim;
    private SimCameraProperties simProperties;
    private double latestCamTimestamp = 0.0;

    /**NOT compatable with sim */
    public Camera(String cameraName, int widthRes, int heightRes, double FOV, Transform3d robotToCam) {
        camera = new PhotonCamera(cameraName);
        poseEstimator = new PhotonPoseEstimator(AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded),
                PoseStrategy.LOWEST_AMBIGUITY, robotToCam);
    }

    /**Compatable with sim */
    public Camera(String cameraName, int widthRes, int heightRes, double FOV, Transform3d robotToCam,
            AprilTagFieldLayout tagLayout, double simLatency, double simLatencyStdDevsMs, double simFPS,
            double simCalibError, double simCalibErrorStdDevs, PoseStrategy poseStrat) {

        if (Utils.isSimulation()) {
            cameraSim = new PhotonCameraSim(new PhotonCamera(cameraName));
            camera = cameraSim.getCamera();
        } else {
            camera = new PhotonCamera(cameraName);
        }
        poseEstimator = new PhotonPoseEstimator(tagLayout, poseStrat, robotToCam);

        simProperties.setCalibError(simCalibError, simCalibErrorStdDevs);
        simProperties.setFPS(simFPS);
        simProperties.setAvgLatencyMs(simLatency);
        simProperties.setLatencyStdDevMs(simLatencyStdDevsMs);
        simProperties.setCalibration(widthRes, heightRes, Rotation2d.fromDegrees(FOV));
    }

    public Optional<EstimatedRobotPose> getEstimatedPose() {
        Optional<EstimatedRobotPose> estimatedPose = Optional.empty();

        for (PhotonPipelineResult change : camera.getAllUnreadResults()) {
            estimatedPose = poseEstimator.update(change);
        }

        latestCamTimestamp = camera.getAllUnreadResults().get(0).getTimestampSeconds();

        return estimatedPose;
    }

    public double getTargetSkew() {
        return camera.getAllUnreadResults().get(0).getBestTarget().getSkew();
    }

    public double getTargetYaw() {
        return camera.getAllUnreadResults().get(0).getBestTarget().getYaw();
    }

    public double getTargetPitch() {
        return camera.getAllUnreadResults().get(0).getBestTarget().getPitch();
    }
}
