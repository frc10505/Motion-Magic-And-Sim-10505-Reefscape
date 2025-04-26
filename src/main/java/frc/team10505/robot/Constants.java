/*
 * Copyright (C) 2025 Team 3602 All rights reserved. This work is
 * licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team10505.robot;

public final class Constants {

  public final class OperatorInterfaceConstants {
    public final static int kXboxControllerPort = 0;
    public final static int kControlPanelPort = 1;
  }

  public final class DrivetrainConstants {
    public final static double rightDriveLaserDistance = 290.0;
    public final static double leftDriveLaserDistance = 290;

    public final static double autonRightDriveLaserDistance = 325.0;
    public final static double autonLeftDriveLaserDistance = 290;

    public final static double closeRightDriveLaserDistance = 235.0; 
    public final static double closeLeftDriveLaserDistance = 250.0;
  }

  public final class AlgaeConstants {
    //Intake speeds
    public final static double ALGAE_INTAKE_NORMAL_SPEED = 0.5;
    public final static double ALGAE_INTAKE_SLOW_SPEED = 0.3;
    public final static double ALGAE_INTKAE_SUPER_SLOW_SPEED = 0.2;

    //pivot angles
    public final static double PIVOT_OUT = 0;
    public final static double PIVOT_UP = 90;
    public final static double PIVOT_DOWN = -90;
  }

  public final class ElevatorConstants {
    //heights
    public final static double ELEV_DOWN = 0;
    public final static double ELEV_L2 = 8;
    public final static double ELEV_L3 = 23.5;
    public final static double ELEV_L4 = 48.5;

    public final static double ELEV_L4_BUMP = 55;
    public final static double ELEV_BARGE = 55.5;

  }

  public final class CoralConstants {
    //speed
    public final static double CORAL_INTAKE_SPEED = 0.37;
    public final static double CORAL_OUTTAKE_SPEED = 0.25;
    public final static double CORAL_SLOW_SPEED = 0.05;
    public final static double CORAL_OUTTAKE_TOP_SPEED = 0.05;

    public final static double CORAL_TROUGH_LEFT_SPEED = 0.5;
    public final static double CORAL_TROUGH_RIGHT_SPEED = 0.225;
    //.4  Donny is skibidi! walter is no buns
  }

}