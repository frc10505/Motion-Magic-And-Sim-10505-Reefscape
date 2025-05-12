package frc.team10505.robot;

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
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team10505.robot.libTypeStuff.Camera;
import static frc.team10505.robot.Constants.VisionConstants.*;

import java.util.Optional;

public class Vision {
    /* Cameras */
    private final PhotonCamera reefCam = new PhotonCamera("reefCam");
    private final PhotonCamera backCam = new PhotonCamera("backCam");

    /* field layout */
    private final AprilTagFieldLayout kFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    public static final int kWidthOfCamera = 4656;
    public static final int kHeightOfCamera = 3496;
    public static final Rotation2d kCameraFOV = Rotation2d.fromDegrees(90.0);

    public static final Transform3d kRobotToBackCamTransform = new Transform3d(
            new Translation3d(-1.0, 1.0, 0.0),
            new Rotation3d(0.0, 0.0, Units.degreesToRadians(180.0)));
    public static final Transform3d kRobotToReefCamTransform = new Transform3d(
            new Translation3d(0.38, -0.35, 0.178),
            new Rotation3d(0.0, Units.degreesToRadians(15.0), Units.degreesToRadians(0.0)));

    /* pose estimators */
    private final PhotonPoseEstimator reefCamEstimator = new PhotonPoseEstimator(kFieldLayout,
            PoseStrategy.LOWEST_AMBIGUITY, kRobotToReefCamTransform);
    private final PhotonPoseEstimator backCamEstimator = new PhotonPoseEstimator(kFieldLayout,
            PoseStrategy.LOWEST_AMBIGUITY, kRobotToBackCamTransform);

    /* Simulation */
    public final VisionSystemSim visionSim = new VisionSystemSim("Vision Sim");
    private final SimCameraProperties cameraProperties = new SimCameraProperties();
    public final PhotonCameraSim reefCamSim = new PhotonCameraSim(reefCam, cameraProperties);
    public final PhotonCameraSim backCamSim = new PhotonCameraSim(backCam, cameraProperties);

    private final Field2d fieldViz = visionSim.getDebugField();

    private final Camera frontCamera = new Camera(FRONT_CAM_NAME, kWidthOfCamera, kHeightOfCamera, kHeightOfCamera, kRobotToBackCamTransform, kFieldLayout, kHeightOfCamera, kHeightOfCamera, kHeightOfCamera, kWidthOfCamera, kHeightOfCamera, null)

    public Vision() {
        cameraProperties.setCalibError(0.25, 0.08);
        cameraProperties.setFPS(20);
        cameraProperties.setAvgLatencyMs(35.0);
        cameraProperties.setLatencyStdDevMs(5);
        cameraProperties.setCalibration(kWidthOfCamera, kHeightOfCamera, kCameraFOV);

        reefCamEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        backCamEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        // sim stuff
        visionSim.addAprilTags(kFieldLayout);
        visionSim.addCamera(reefCamSim, kRobotToReefCamTransform);
        visionSim.addCamera(backCamSim, kRobotToBackCamTransform);

        SmartDashboard.putData("Field Viz", fieldViz);
    }

    /* Calculations for pose estimations */
    public double lastReefCamEstimateTimestamp = 0.0;

    public Optional<EstimatedRobotPose> getReefCamEstimatedPose() {
        Optional<EstimatedRobotPose> reefCamRobotPose = Optional.empty();

        for (PhotonPipelineResult change : reefCam.getAllUnreadResults()) {

            reefCamRobotPose = reefCamEstimator.update(change);
        }

        lastReefCamEstimateTimestamp = reefCam.getLatestResult().getTimestampSeconds();

        return reefCamRobotPose;
    }

    public double simLastReefCamEstimateTimestamp = 0.0;

    public Optional<EstimatedRobotPose> simGetReefCamEstimatedPose() {
        Optional<EstimatedRobotPose> simReefCamRobotPose = Optional.empty();

        for (PhotonPipelineResult change : reefCamSim.getCamera().getAllUnreadResults()) {

            simReefCamRobotPose = reefCamEstimator.update(change);
        }

        simLastReefCamEstimateTimestamp = reefCamSim.getCamera().getLatestResult().getTimestampSeconds();

        return simReefCamRobotPose;
    }

    public double getTargetSkew(PhotonCameraSim camera) {
        try {
            return camera.getCamera().getLatestResult().getBestTarget().getSkew();
        } catch (Exception e) {
            return 0.0;
        }
    }

    public double getTargetYaw(PhotonCameraSim camera) {
        try {
            return camera.getCamera().getLatestResult().getBestTarget().getYaw();
        } catch (Exception e) {
            return 0.0;
        }
    }

    public double getTargetPitch(PhotonCameraSim camera) {
        try {
            return camera.getCamera().getLatestResult().getBestTarget().getPitch();
        } catch (Exception e) {
            return 0.0;
        }
    }

    public Transform3d getTargetTransformation(PhotonCameraSim camera) {
        try {
            return camera.getCamera().getLatestResult().getBestTarget().getBestCameraToTarget();
        } catch (Exception e) {
            return new Transform3d();
        }
    }

    // public double lastBackCamEstimateTimestamp = 0.0;

    // public Optional<EstimatedRobotPose> getBackCamEstimatedPose(){
    // Optional<EstimatedRobotPose> backCamRobotPose = Optional.empty();

    // for (PhotonPipelineResult change : backCam.getAllUnreadResults() ){

    // backCamRobotPose = backCamEstimator.update(change);
    // }

    // lastBackCamEstimateTimestamp =
    // backCam.getLatestResult().getTimestampSeconds();

    // return backCamRobotPose;
    // }

    public Pose2d latestVisionPose = new Pose2d();

    public void updateViz(Pose2d pose) {
        // visionSim.getDebugField();
        // visionSim.update(pose);
        fieldViz.setRobotPose(pose);

    }

    public void reset() {
        visionSim.clearAprilTags();
        visionSim.addAprilTags(kFieldLayout);

    }

    public Pose2d getLatestPose() {
        return reefCamEstimator.getReferencePose().toPose2d();
    }

    // @Override
    public void updateDashboard() {

        getReefCamEstimatedPose().ifPresent(est -> {
            SmartDashboard.putNumber("reef cam pose x", est.estimatedPose.toPose2d().getX());
            SmartDashboard.putNumber("reef cam pose y", est.estimatedPose.toPose2d().getY());
            SmartDashboard.putNumber("reef cam pose rot", est.estimatedPose.toPose2d().getRotation().getDegrees());

            SmartDashboard.putNumber("reef cam target transform x", getTargetTransformation(reefCamSim).getX());
            SmartDashboard.putNumber("reef cam target transform y", getTargetTransformation(reefCamSim).getY());
            SmartDashboard.putNumber("reef cam target transform z", getTargetTransformation(reefCamSim).getZ());
            SmartDashboard.putNumber("reef cam target rotation x",
                    getTargetTransformation(reefCamSim).getRotation().getX());
            SmartDashboard.putNumber("reef cam target rotation y",
                    getTargetTransformation(reefCamSim).getRotation().getY());
            SmartDashboard.putNumber("reef cam target rotation z",
                    getTargetTransformation(reefCamSim).getRotation().getZ());
            SmartDashboard.putNumber("reef cam target ID",
                    reefCamSim.getCamera().getLatestResult().getBestTarget().fiducialId);

            SmartDashboard.putNumber("back cam target transform x", getTargetTransformation(backCamSim).getX());
            SmartDashboard.putNumber("back cam target transform y", getTargetTransformation(backCamSim).getY());
            SmartDashboard.putNumber("back cam target transform z", getTargetTransformation(backCamSim).getZ());
            SmartDashboard.putNumber("back cam target rotation x",
                    getTargetTransformation(backCamSim).getRotation().getX());
            SmartDashboard.putNumber("back cam target rotation y",
                    getTargetTransformation(backCamSim).getRotation().getY());
            SmartDashboard.putNumber("back cam target rotation z",
                    getTargetTransformation(backCamSim).getRotation().getZ());
            SmartDashboard.putNumber("back cam target ID",
                    backCamSim.getCamera().getLatestResult().getBestTarget().fiducialId);

        });

    }

}
