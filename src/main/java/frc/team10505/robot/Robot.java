/*
 * Copyright (C) 2025 Team 10505 All rights reserved. This work is
 * licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team10505.robot;

import edu.wpi.first.hal.ThreadsJNI;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import com.ctre.phoenix6.Utils;

import au.grapplerobotics.CanBridge;

public class Robot extends TimedRobot {

  private final RobotContainer robotContainer = new RobotContainer();
  private Command autonomousCommand;

  private Thread visionThread = new Thread(() -> robotContainer.updateVisionPose());

  public Robot() {
    CanBridge.runTCP();
  }

  @Override
  public void robotInit() {
    //visionThread.setDaemon(true);
    //visionThread.start();

  }

  @Override
  public void robotPeriodic() {
   Threads.setCurrentThreadPriority(true, 99);
    //visionThread.setPriority(Thread.MIN_PRIORITY);
    CommandScheduler.getInstance().run();

   //visionThread.run();

    //robotContainer.updatePose();
    if (Utils.isSimulation()) {
      robotContainer.updateVisionPose();
    }

   Threads.setCurrentThreadPriority(false, 10);
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
    if (Utils.isSimulation()) {
      autonomousCommand = robotContainer.getAutonomousCommand();
      if (autonomousCommand != null) {
        autonomousCommand.schedule();
      }
    }
  }

  @Override
  public void testPeriodic() {
  }

}