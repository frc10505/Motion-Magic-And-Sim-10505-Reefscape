package frc.team10505.robot.Vision;

//import frc.team10505.robot.LimelightHelpers;
import static frc.team10505.robot.Constants.VisionConstants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {





    public double tx = LimelightHelpers.getTX(FRONT_LL_NAME);
    public double ta = LimelightHelpers.getTA(FRONT_LL_NAME);
    public double ty = LimelightHelpers.getTY(FRONT_LL_NAME);
    public double txnc = LimelightHelpers.getTXNC(FRONT_LL_NAME);
    public double tync = LimelightHelpers.getTYNC(FRONT_LL_NAME);
    public boolean hasTarget = LimelightHelpers.getTV(FRONT_LL_NAME);

    public Limelight(){

    }

    @Override
    public void periodic(){
        
    }
}
