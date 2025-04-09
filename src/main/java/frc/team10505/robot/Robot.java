/*
 * Copyright (C) 2025 Team 10505 All rights reserved. This work is
 * licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team10505.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import com.ctre.phoenix6.Utils;

import au.grapplerobotics.CanBridge;

public class Robot extends TimedRobot {

  private final RobotContainer robotContainer = new RobotContainer();

  private Command autonomousCommand;

  public Robot() {
    CanBridge.runTCP();
    //CameraServer.startAutomaticCapture();
  }

  @Override
  public void robotInit() {
   // robotContainer.cameraFeedInit();
   if(Utils.isSimulation()){
   robotContainer.vizSimInit();}
  //  robotContainer.updateVizSim();
  }



  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    robotContainer.updatePose();
    robotContainer.updateDriveSensors();
    if(Utils.isSimulation()){
    robotContainer.updateVizSim();}
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
    autonomousCommand = robotContainer.getAutonomousCommand();
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  @Override
  public void testPeriodic() {
  }

}