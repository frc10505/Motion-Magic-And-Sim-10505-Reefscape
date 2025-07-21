/*
 * Copyright (C) 2025 Team 3602 All rights reserved. This work is
 * licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team3602.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public final class Constants {
  /**All constants that have to do with hardware & hardware configurations */
  public final class HardwareConstants{
    //THE CURRENT LIMITS AND CAN IDS FOR THE DRIVETRAIN CAN BE FOUND IN TunerConstants.java
    /*40-51 can ids are used in the drivetrain */
  }

  public class VisionConstants{    
    public final static String FRONT_CAM_NAME = "Front Cam";
    public final static int FRONT_CAM_WIDTH_RES = 4656;
    public final static int FRONT_CAM_HEIGHT_RES = 3496;
    public final static double FRONT_CAM_FOV_DEG = 90;
    public final static Transform3d ROBOT_TO_FRONT_CAM = new Transform3d(
            new Translation3d(0.33, 0.33, 0.02),
            new Rotation3d(0.0, Units.degreesToRadians(15.0), Units.degreesToRadians(-15.0)));
    
    public final static String BACK_CAM_NAME = "Back Cam";
    public final static int BACK_CAM_WIDTH_RES = 4656;
    public final static int BACK_CAM_HEIGHT_RES = 3496;
    public final static double BACK_CAM_FOV_DEG = 90;
    public final static Transform3d ROBOT_TO_BACK_CAM= new Transform3d(
      new Translation3d(0.-38, 0.0, 0.02),
      new Rotation3d(0.0, Units.degreesToRadians(15.0), Units.degreesToRadians(180.0)));

  }  
}