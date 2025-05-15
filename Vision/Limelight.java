package frc.team10505.robot;

import org.photonvision.simulation.VisionSystemSim;

import static frc.team10505.robot.Constants.VisionConstants.*;
import static frc.team10505.robot.LimelightHelpers.*;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Limelight {//} extends SubsystemBase {

    public double tx = getTX(FRONT_CAM_NAME);
    public double ta = LimelightHelpers.getTA(FRONT_CAM_NAME);
    public double ty = LimelightHelpers.getTY(FRONT_CAM_NAME);
    public double txnc = LimelightHelpers.getTXNC(FRONT_CAM_NAME);
    public double tync = LimelightHelpers.getTYNC(FRONT_CAM_NAME);
    public boolean hasTarget = LimelightHelpers.getTV(FRONT_CAM_NAME);

    public Pose2d bruh = getBotPose2d_wpiBlue(FRONT_CAM_NAME);

    
    public Limelight(){ 
    }

    public Pose2d getPose(String cameraName){
        return getBotPose2d_wpiBlue(cameraName);
    }

    public void updateDashboard(){
        SmartDashboard.putNumber("Back LL Pose X", getBotPose2d_wpiBlue(BACK_CAM_NAME).getX());
        SmartDashboard.putNumber("Back LL Pose Y", getBotPose2d_wpiBlue(BACK_CAM_NAME).getY());
        SmartDashboard.putNumber("Back LL Pose rot", getBotPose2d_wpiBlue(BACK_CAM_NAME).getRotation().getDegrees());
        SmartDashboard.putNumber("Front LL Pose X", getBotPose2d_wpiBlue(FRONT_CAM_NAME).getX());
        SmartDashboard.putNumber("Front LL Pose Y", getBotPose2d_wpiBlue(FRONT_CAM_NAME).getY());
        SmartDashboard.putNumber("Front LL Pose rot", getBotPose2d_wpiBlue(FRONT_CAM_NAME).getRotation().getDegrees());
    }
}