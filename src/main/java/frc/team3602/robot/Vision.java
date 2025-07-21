package frc.team3602.robot;

import org.opencv.photo.Photo;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.team3602.robot.Constants.VisionConstants.*;

import java.util.Optional;

/**
 * Vision class using photon vision
 * Author: C Furmanski (clydestale158)
 */
public class Vision {
    /* April tag layout */
    private final AprilTagFieldLayout tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    /* Photon camera vars */
    public final PhotonCamera frontCam;
    private final PhotonPoseEstimator frontCamPE;// pose estimator
    public double frontCamLastPoseTS = 0;// latest pose timestamp
    private PhotonPipelineResult frontCamLastRes = new PhotonPipelineResult(); // latest result

    public double frontCamSkew = 0;
    public double frontCamYaw = 0;
    public double frontCamPitch = 0;
    public Pose2d frontCamEstimatedPose = new Pose2d();

    public final PhotonCamera backCam;
    private final PhotonPoseEstimator backCamPE;// pose estimator
    public double backCamLastPoseTS = 0;// latest pose timestamp
    private PhotonPipelineResult backCamLastRes = new PhotonPipelineResult(); // latest result

    public double backCamSkew = 0;
    public double backCamPitch = 0;
    public double backCamYaw = 0;
    public Pose2d backCamEstimatedPose = new Pose2d();

    /* Sim vars */
    public VisionSystemSim visionSim = new VisionSystemSim("Vision Sim");
    private Field2d fieldViz = visionSim.getDebugField();

    private PhotonCameraSim frontCamSim;
    private PhotonCameraSim backCamSim;
    // private SimCameraProperties simProps;

    /** Constructor, supports sim */
    public Vision() {
        if (RobotBase.isSimulation()) {
            // simProps = new SimCameraProperties();
            // simProps.setCalibError(0.25, 0.08);
            // simProps.setCalibration(4656, 3496, new Rotation2d(90));
            // simProps.setFPS(20);
            // simProps.setAvgLatencyMs(35);
            // simProps.setLatencyStdDevMs(5);
            // //TODO apply sim props

            frontCamSim = new PhotonCameraSim(new PhotonCamera(FRONT_CAM_NAME));
            frontCam = frontCamSim.getCamera();

            backCamSim = new PhotonCameraSim(new PhotonCamera(BACK_CAM_NAME));
            backCam = backCamSim.getCamera();

            visionSim.addAprilTags(tagLayout);
            visionSim.addCamera(frontCamSim, ROBOT_TO_FRONT_CAM);
            visionSim.addCamera(backCamSim, ROBOT_TO_BACK_CAM);

            // SmartDashboard.putData("Vision Sim", visionSim.getDebugField());
            SmartDashboard.putData("Field Viz", fieldViz);
        } else {
            frontCam = new PhotonCamera(FRONT_CAM_NAME);
            backCam = new PhotonCamera(BACK_CAM_NAME);
        }

        frontCamPE = new PhotonPoseEstimator(tagLayout, PoseStrategy.LOWEST_AMBIGUITY, ROBOT_TO_FRONT_CAM);
        backCamPE = new PhotonPoseEstimator(tagLayout, PoseStrategy.LOWEST_AMBIGUITY, ROBOT_TO_BACK_CAM);
    }

    public Optional<EstimatedRobotPose> getFrontCamEstimatedPose() {
        Optional<EstimatedRobotPose> estimatedPose = Optional.empty();

        for (PhotonPipelineResult change : frontCam.getAllUnreadResults()) {
            estimatedPose = frontCamPE.update(frontCamLastRes);
            frontCamLastPoseTS = frontCam.getAllUnreadResults().get(frontCam.getAllUnreadResults().lastIndexOf(change))
                    .getTimestampSeconds();
        }

        return estimatedPose;
    }

    public Optional<EstimatedRobotPose> getBackCamEstimatedPose() {
        Optional<EstimatedRobotPose> estimatedPose = Optional.empty();

        for (PhotonPipelineResult change : backCam.getAllUnreadResults()) {
            estimatedPose = backCamPE.update(backCamLastRes);
            backCamLastPoseTS = backCam.getAllUnreadResults().get(backCam.getAllUnreadResults().lastIndexOf(change))
                    .getTimestampSeconds();
        }

        return estimatedPose;
    }

    public double getTargetSkew(PhotonPipelineResult latestResult, PhotonCamera camera) {
        return camera.getAllUnreadResults().get(camera.getAllUnreadResults().lastIndexOf(latestResult)).getBestTarget()
                .getSkew();
    }

    public double getTargetYaw(PhotonPipelineResult latestResult, PhotonCamera camera) {
        return camera.getAllUnreadResults().get(camera.getAllUnreadResults().lastIndexOf(latestResult))
                .getBestTarget().getYaw();

    }

    public double getTargetPitch(PhotonPipelineResult latestResult, PhotonCamera camera) {
        return camera.getAllUnreadResults().get(camera.getAllUnreadResults().lastIndexOf(latestResult)).getBestTarget()
                .getPitch();
    }

    public Transform3d getTargetTransformation(PhotonPipelineResult latestResult, PhotonCamera camera) {
        return camera.getAllUnreadResults().get(camera.getAllUnreadResults().lastIndexOf(latestResult))
                .getBestTarget().bestCameraToTarget;
    }

    /** Must be called periodically in simulations */
    public void updateViz(Pose2d pose) {
        visionSim.update(pose);
        visionSim.getDebugField();
    }

    public void reset() {
        visionSim.clearAprilTags();
        visionSim.addAprilTags(tagLayout);
    }

    /**
     * gives data to SmartDashboard for the estimated robot pose's x, y, and
     * rotational values, and latest timestamp
     */
    public void putPEValues(Optional<EstimatedRobotPose> pose, PhotonCamera camera) {
        pose.ifPresent(est -> {
            SmartDashboard.putNumber(camera.getName() + " Est Robot Pose X", est.estimatedPose.toPose2d().getX());
            SmartDashboard.putNumber(camera.getName() + " Est Robot Pose Y", est.estimatedPose.toPose2d().getY());
            SmartDashboard.putNumber(camera.getName() + " Est Robot Pose Rot",
                    est.estimatedPose.toPose2d().getRotation().getDegrees());
            SmartDashboard.putNumber("Front Cam Est Robot Pose Latest Timestamp", frontCamLastPoseTS);
            SmartDashboard.putNumber("Front Cam Est Robot Pose Latest Timestamp", backCamLastPoseTS);
        });

        if (pose.isEmpty()) {
            SmartDashboard.putString("Errors", camera.getName() + " est pose is empty!");
        }
    }

    /**
     * gives data to SmartDashboard for the best target's x, y, z, rot x, rot y, rot
     * z, and fiducial id
     */
    public void putTargetValues(PhotonCamera camera) {
        try {
            for (PhotonPipelineResult change : camera.getAllUnreadResults()) {
                SmartDashboard.putNumber(camera + "target transform x",
                        getTargetTransformation(change, camera).getX());
                SmartDashboard.putNumber(camera + " target transform y",
                        getTargetTransformation(change, camera).getY());
                SmartDashboard.putNumber(camera + " target transform z",
                        getTargetTransformation(change, camera).getZ());
                SmartDashboard.putNumber(camera + " target rotation x",
                        getTargetTransformation(change, camera).getRotation().getX());
                SmartDashboard.putNumber(camera + " target rotation y",
                        getTargetTransformation(change, camera).getRotation().getY());
                SmartDashboard.putNumber(camera + " target rotation z",
                        getTargetTransformation(change, camera).getRotation().getZ());
                SmartDashboard.putNumber(camera + " target ID",
                        camera.getAllUnreadResults().get(0).getBestTarget().fiducialId);
            }
        } catch (Exception ex) {
            SmartDashboard.putString("Errors", camera.getName() + " target value is empty!");
        }
    }

    public void updatePoseEstimations() {
        getFrontCamEstimatedPose().ifPresent(est -> {
            frontCamEstimatedPose = est.estimatedPose.toPose2d();
        });

        if (getFrontCamEstimatedPose().isEmpty()) {
            SmartDashboard.putString("Errors", "Front Cam est pose is empty!");
        }

        getBackCamEstimatedPose().ifPresent(est -> {
            backCamEstimatedPose = est.estimatedPose.toPose2d();
        });

        if (getBackCamEstimatedPose().isEmpty()) {
            SmartDashboard.putString("Errors", "Back Cam est pose is empty!");
        }
    }

    /** Must be called periodically to give vision functionality */
    public void updateResults() {
        for (PhotonPipelineResult newResult : frontCam.getAllUnreadResults()) {
            frontCamLastRes = newResult;
        }
        for (PhotonPipelineResult newResult : backCam.getAllUnreadResults()) {
            backCamLastRes = newResult;
        }
    }
}
