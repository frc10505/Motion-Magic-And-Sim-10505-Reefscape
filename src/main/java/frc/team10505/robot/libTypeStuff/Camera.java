package frc.team10505.robot.libTypeStuff;

import java.util.Optional;

import org.opencv.core.Size;
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Camera {
    private final PhotonCamera camera;
    private final PhotonPoseEstimator poseEstimator;
    private PhotonCameraSim cameraSim;
    private SimCameraProperties simProperties;
    private String cameraName;
    private double latestPoseTimestamp = 0.0;

    /**NOT compatable with sim */
    public Camera(String cameraName, int widthRes, int heightRes, double FOV, Transform3d robotToCam) {
        camera = new PhotonCamera(cameraName);
        poseEstimator = new PhotonPoseEstimator(AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded),
                PoseStrategy.LOWEST_AMBIGUITY, robotToCam);
        this.cameraName = cameraName;
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
        simProperties = new SimCameraProperties();

        simProperties.setCalibError(simCalibError, simCalibErrorStdDevs);
        simProperties.setFPS(simFPS);
        simProperties.setAvgLatencyMs(simLatency);
        simProperties.setLatencyStdDevMs(simLatencyStdDevsMs);
        simProperties.setCalibration(widthRes, heightRes, Rotation2d.fromDegrees(FOV));

        this.cameraName = cameraName;
    }

    public Optional<EstimatedRobotPose> getEstimatedPose() {
        Optional<EstimatedRobotPose> estimatedPose = Optional.empty();

        for (PhotonPipelineResult change : camera.getAllUnreadResults()) {
            estimatedPose = poseEstimator.update(change);
            latestPoseTimestamp = camera.getAllUnreadResults().get(camera.getAllUnreadResults().lastIndexOf(change)).getTimestampSeconds();
        }       

        return estimatedPose;
    }

    public double getTargetSkew() {
        return camera.getAllUnreadResults().get(-1).getBestTarget().getSkew();
    }

    public double getTargetYaw() {
        return camera.getAllUnreadResults().get(camera.getAllUnreadResults().lastIndexOf(camera.getAllUnreadResults())).getBestTarget().getYaw();
        
    }

    public double getTargetPitch() {
        return camera.getAllUnreadResults().get(camera.getPipelineIndex()).getBestTarget().getPitch();
    }

    public PhotonCameraSim getCameraSim(){
        return cameraSim;
    }

    public Transform3d getTargetTransformation(PhotonPipelineResult change){
        return camera.getAllUnreadResults().get(camera.getAllUnreadResults().lastIndexOf(change)).getBestTarget().bestCameraToTarget;
    }

    /**gives data to SmartDashboard for the estimated robot pose's x, y, and rotational values, and latest timestamp */
    public void putRobotPoseValues(){
        getEstimatedPose().ifPresent(est -> {
            SmartDashboard.putNumber(cameraName + " Est Robot Pose X", est.estimatedPose.toPose2d().getX());
            SmartDashboard.putNumber(cameraName + " Est Robot Pose Y", est.estimatedPose.toPose2d().getY());
            SmartDashboard.putNumber(cameraName + " Est Robot Pose Rot", est.estimatedPose.toPose2d().getRotation().getDegrees());
            SmartDashboard.putNumber(cameraName + " Est Robot Pose Latest Timestamp", latestPoseTimestamp);

        });
    }

    /**gives data to SmartDashboard for the best target's x, y, z, rot x, rot y, rot z, and fiducial id */
    public void putTargetValues(){
            for(PhotonPipelineResult change : camera.getAllUnreadResults()){
                SmartDashboard.putNumber(cameraName + "target transform x",
            getTargetTransformation(change).getX());            
             SmartDashboard.putNumber(cameraName +" target transform y", getTargetTransformation(change).getY());
            SmartDashboard.putNumber(cameraName +" target transform z", getTargetTransformation(change).getZ());
            SmartDashboard.putNumber(cameraName +" target rotation x",
                    getTargetTransformation(change).getRotation().getX());
            SmartDashboard.putNumber(cameraName +" target rotation y",
                    getTargetTransformation(change).getRotation().getY());
            SmartDashboard.putNumber(cameraName +" target rotation z",
                    getTargetTransformation(change).getRotation().getZ());
            SmartDashboard.putNumber(cameraName +" target ID",
                    camera.getAllUnreadResults().get(0).getBestTarget().fiducialId);
    }
}
}
