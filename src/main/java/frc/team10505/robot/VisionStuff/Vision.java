package frc.team10505.robot.VisionStuff;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.VisionSystemSim;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import static frc.team10505.robot.Constants.VisionConstants.*;

public class Vision {
    /* field layout */
    private final AprilTagFieldLayout kFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    /* Simulation */
    public final VisionSystemSim visionSim = new VisionSystemSim("Vision Sim");

    private final Field2d fieldViz = visionSim.getDebugField();

    public final Camera frontCamera = new Camera(FRONT_CAM_NAME, FRONT_CAM_WIDTH_RES, FRONT_CAM_HEIGHT_RES, FRONT_CAM_FOV_DEG, FRONT_CAM_TO_ROBOT, kFieldLayout, 35, 5, 20, 0.25, 0.08, PoseStrategy.LOWEST_AMBIGUITY);
    
    public final Camera backCamera = new Camera(BACK_CAM_NAME, BACK_CAM_WIDTH_RES, BACK_CAM_HEIGHT_RES, BACK_CAM_FOV_DEG, BACK_CAM_TO_ROBOT, kFieldLayout, 35, 5, 20, 0.25, 0.08, PoseStrategy.LOWEST_AMBIGUITY);
    
    
    public Vision(){
        // sim stuff
        visionSim.addAprilTags(kFieldLayout);
        visionSim.addCamera(frontCamera.getCameraSim(), FRONT_CAM_TO_ROBOT);
        visionSim.addCamera(backCamera.getCameraSim(), BACK_CAM_TO_ROBOT);

        SmartDashboard.putData("Field Viz", fieldViz);
    }

    public void updateViz(Pose2d pose) {
        fieldViz.setRobotPose(pose);
    } 

    public void reset() {
        visionSim.clearAprilTags();
        visionSim.addAprilTags(kFieldLayout);
    }

    public void updateDashboard() {
        frontCamera.putRobotPoseValues();
        frontCamera.putTargetValues();
        backCamera.putRobotPoseValues();
        backCamera.putTargetValues();
    }

}
