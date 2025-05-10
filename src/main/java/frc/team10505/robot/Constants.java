/*
 * Copyright (C) 2025 Team 3602 All rights reserved. This work is
 * licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team10505.robot;

public final class Constants {
  /**All constants that have to do with hardware & hardware configurations */
  public final class HardwareConstants{
    //THE CURRENT LIMITS AND CAN IDS FOR THE DRIVETRAIN CAN BE FOUND IN TunerConstants.java
   
    /*Robo Rio ports/channels */
    public final static int DRIVETRAIN_BLINKY_LIGHT_CHANNEL = 0;

    /*CAN IDS */
    public final static int CORAL_LEFT_MOTOR_ID = 2;
    public final static int CORAL_RIGHT_MOTOR_ID = 3;

    public final static int ALGAE_INTAKE_MOTOR_ID = 7;
    public final static int ALGAE_PIVOT_MOTOR_ID = 8;

    public final static int ELEVATOR_MOTOR_ID = 10;
    public final static int ELEVATOR_FOLLOWER_MOTOR_ID = 11;

    /*40-51 can ids are used in the drivetrain */

    public final static int DRIVETRAIN_RIGHT_LASER_ID = 52;
    public final static int DRIVETRAIN_LEFT_LASER_ID = 53;
   
    public final static int CORAL_IN_LASER_ID = 60;
    public final static int CORAL_OUT_LASER_ID = 61;

    /*Current Limits */   
    public final static int ALGAE_PIVOT_MOTOR_CURRENT_LIMIT = 15;
    public final static int ALGAE_INTAKE_MOTOR_CURRENT_LIMIT = 25;

    public final static int CORAL_MOTOR_CURRENT_LIMIT = 15;

    public final static int ELEVATOR_MOTOR_CURRENT_LIMIT = 40;

    /*Encoder Offsets */
    public final static double ALGAE_PIVOT_ENCODER_OFFSET = 0;
    public final static double ALGAE_PIVOT_ENCODER_SCALE = 360;

    /*Gearstacks */
    public final static double ELEVATOR_GEARSTACK = 12;
    public final static double ALGAE_PIVOT_GEARSTACK = 80;
  }

    public final static double rightDriveLaserDistance = 290.0;
    public final static double leftDriveLaserDistance = 290;

    public final static double autonRightDriveLaserDistance = 325.0;
    public final static double autonLeftDriveLaserDistance = 290;

    public final static double closeRightDriveLaserDistance = 235.0; 
    public final static double closeLeftDriveLaserDistance = 250.0;

    public final static double DRIVE_TELEOP_LEFT_SPEED = 0.4;
    public final static double DRIVE_TELEOP_RIGHT_SPEED = 0.6;

  //Algae constants
    //Intake speeds
    public final static double ALGAE_INTAKE_NORMAL_SPEED = 0.5;
    public final static double ALGAE_INTAKE_SLOW_SPEED = 0.3;
    public final static double ALGAE_INTKAE_SUPER_SLOW_SPEED = 0.2;

    //pivot angles
    public final static double ALGAE_PIVOT_OUT = 0;
    public final static double ALGAE_PIVOT_UP = 90;
    public final static double ALGAE_PIVOT_DOWN = -90;


  //Elevator constants
    public final static double ELEV_DOWN = 0;
    public final static double ELEV_L2 = 8;
    public final static double ELEV_L3 = 23.5;
    public final static double ELEV_L4 = 48.5;

    public final static double ELEV_L4_BUMP = 55;
    public final static double ELEV_BARGE = 55.5;


  //Coral constants
    public final static double CORAL_INTAKE_SPEED = 0.37;
    public final static double CORAL_OUTTAKE_SPEED = 0.25;
    public final static double CORAL_SLOW_SPEED = 0.05;
    public final static double CORAL_OUTTAKE_TOP_SPEED = 0.05;

    public final static double CORAL_TROUGH_LEFT_SPEED = 0.5;
    public final static double CORAL_TROUGH_RIGHT_SPEED = 0.225;
    //Donny is skibidi! walter is no buns
}