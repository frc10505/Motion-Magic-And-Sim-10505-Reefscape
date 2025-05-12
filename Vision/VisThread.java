package frc.team10505.robot.Vision;

import edu.wpi.first.wpilibj.IterativeRobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.team10505.robot.Vision;

public class VisThread extends TimedRobot{
    private Vision vision = new Vision();

    public VisThread(){
    }

    
    @Override
    public void disabledInit() {
    }
    
    @Override
    public void disabledPeriodic() {
        
        vision.getLatestPose();
    }

    @Override
    public void disabledExit(){

    }

     
    @Override
    public void teleopInit() {
    }
    
    @Override
    public void teleopPeriodic() {
        vision.getLatestPose();
    }

    @Override
    public void teleopExit(){
        
    }


    @Override
    public void autonomousInit() {
    }
    
    @Override
    public void autonomousPeriodic() {
        vision.getLatestPose();
    }

    @Override
    public void autonomousExit(){
        
    }


    @Override
    public void testInit() {
    }
    
    @Override
    public void testPeriodic() {
        vision.getLatestPose();
    }

    @Override
    public void testExit(){
        
    }
     
}
