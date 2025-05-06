/*
 * Copyright (C) 2025 Team 10505 All rights reserved. This work is
 * licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team10505.robot;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.team10505.robot.subsystems.DrivetrainSubsystem;
import frc.team10505.robot.subsystems.ElevatorSubsystem;
import frc.team10505.robot.subsystems.AlgaeSubsystem;
import frc.team10505.robot.subsystems.CoralSubsystem;
import static frc.team10505.robot.Constants.AlgaeConstants.*;
import static frc.team10505.robot.Constants.ElevatorConstants.*;
import static frc.team10505.robot.Constants.CoralConstants.*;

public class Superstructure {
    /*variables */
    private CoralSubsystem coralSubsystem;
    private AlgaeSubsystem algaeSubsystem;
    private ElevatorSubsystem elevatorSubsystem;
    private DrivetrainSubsystem drivetrainSubsystem;

    private SwerveRequest.ApplyRobotSpeeds autoRobotDrive = new SwerveRequest.ApplyRobotSpeeds();

    /*Constructor */
    public Superstructure(CoralSubsystem coralSubsys, AlgaeSubsystem algaeSubsys, ElevatorSubsystem elevatorSubsys,
            DrivetrainSubsystem drivetrainSubsys) {
        this.coralSubsystem = coralSubsys;
        this.algaeSubsystem = algaeSubsys;
        this.elevatorSubsystem = elevatorSubsys;
        this.drivetrainSubsystem = drivetrainSubsys;
    }

    public Command intakeCoral() {
        return coralSubsystem.slowEndIntake(CORAL_INTAKE_SPEED).until(() -> (coralSubsystem.outSensor() && coralSubsystem.inSensor()));
    }

    public Command outputCoral() {
        return Commands.sequence(
                coralSubsystem.runIntake(CORAL_OUTTAKE_SPEED).until(() -> (!coralSubsystem.outSensor())),
                elevatorSubsystem.setHeight(0.0));
    }

    public Command outputCoralTrough() {
        return coralSubsystem.trough().until(() -> (!coralSubsystem.outSensor()));    
    }

    public Command outputTopCoral() {
        return Commands.sequence(
                coralSubsystem.runIntake(CORAL_OUTTAKE_TOP_SPEED),
                Commands.waitSeconds(0.5),
                elevatorSubsystem.setHeight(ELEV_L4_BUMP),
                Commands.waitUntil(() -> (elevatorSubsystem.isAbove(52.0))), 
                Commands.waitSeconds(0.2),
                coralSubsystem.setIntake(0),
                elevatorSubsystem.setHeight(ELEV_DOWN));
    }

    public Command bombsAway() {
        return Commands.sequence(
                elevatorSubsystem.setHeight(ELEV_BARGE),
                Commands.waitUntil(() -> (elevatorSubsystem.getElevatorEncoder() > 42.5))
        );
    }

    public Command detonate() {
        return Commands.sequence(

                algaeSubsystem.setVoltage(-1.5).withTimeout(0.05),
                algaeSubsystem.setVoltage(5.0).until(() -> algaeSubsystem.getPivotEncoder() > 50),
                algaeSubsystem.setIntake(0.3),
                algaeSubsystem.setAngle(90)

        );
    }

    // public Command regurgitateAlgae(){
    // return Commands.sequence(
    // algaeSubsystem.intakeForwardSlower().withTimeout(0.6),
    // algaeSubsystem.intakeStop()
    // );
    // }

    public Command takeCover() {
        return Commands.sequence(
                elevatorSubsystem.setHeight(0.0),
                algaeSubsystem.setAngle(-90),
                algaeSubsystem.stopIntake()

        );

    }

    // public Command grabAlgae() {
    // return Commands.sequence(
    // algaeSubsystem.intakeReverse(),
    // algaeSubsystem.coastPivot());
    // }

    public Command holdAlgae() {
        return Commands.sequence(
                algaeSubsystem.stopIntake(),
                algaeSubsystem.setAngle(-13));
    }

    public Command manualL4Bump() {
        return Commands.sequence(
                elevatorSubsystem.setHeight(55.0), // 54.5 //54 BADISH
                // Commands.waitUntil(() -> elevatorSubsystem.isNearGoal()),
                Commands.waitUntil(() -> (elevatorSubsystem.isAbove(52.0))),
                Commands.waitSeconds(0.2),
                coralSubsystem.setIntake(0),
                elevatorSubsystem.setHeight(0.0));
    }

    public Command seekLeftIntakeStation(){
        return parallel(
            coralSubsystem.runIntake(CORAL_INTAKE_SPEED).unless(() -> coralSubsystem.inSensor()),
            drivetrainSubsystem.goToLeftStation()
        );
    }

    // COMMANDS FOR AUTONS
    public Command autoIntakeCoral() {
        return coralSubsystem.slowEndIntake(CORAL_INTAKE_SPEED).until(() -> (coralSubsystem.outSensor() && coralSubsystem.inSensor()));
    }

    public Command autoOutputCoral() {
        return coralSubsystem.runIntake(0.25).until(() -> (!coralSubsystem.outSensor()));
    }

    public Command autoOutputCoralTrough() {
        return coralSubsystem.trough().until(() -> (!coralSubsystem.outSensor()));
    }

    public Command autoOutputTopCoral() {
        return coralSubsystem.runIntake(0.2).until(() -> (!coralSubsystem.outSensor()));
    }

    public Command autoHoldAlgae() {
        return algaeSubsystem.stopIntake();
    }

    public Command autoScoreCoralL4() {
        return Commands.sequence(
                elevatorSubsystem.setHeight(ELEV_L4), 
                Commands.waitUntil(() -> elevatorSubsystem.isNearGoal()),
                coralSubsystem.setIntake(CORAL_INTAKE_SPEED),//TODO figure out what the gaf is this .37? 
                //NOTE are we stupid? probably....... :(
                //other auto scores use .25,  and i think teleop l4 scoring uses 0.05
                Commands.race(
                        Commands.waitUntil(() -> !coralSubsystem.outSensor()), 
                        Commands.waitSeconds(1.2)));
    }

    public Command autoL4Bump() {
        return Commands.sequence(
                elevatorSubsystem.setHeight(ELEV_L4_BUMP), 
                Commands.waitUntil(() -> (elevatorSubsystem.isAbove(52.0))),
                Commands.waitSeconds(0.2),
                coralSubsystem.setIntake(0));
    }

    public Command autoScoreCoralL2() {
        return Commands.sequence(
                elevatorSubsystem.setHeight(ELEV_L2),
                Commands.waitUntil(() -> elevatorSubsystem.isNearGoal()),
                coralSubsystem.setIntake(CORAL_OUTTAKE_SPEED),
                Commands.race(
                        Commands.waitUntil(() -> (!coralSubsystem.outSensor())),
                        Commands.waitSeconds(2.5)),
                coralSubsystem.setIntake(0));
    }

    public Command autoScoreCoralL1() {
        return Commands.sequence(
                elevatorSubsystem.setHeight(ELEV_DOWN),
                Commands.waitUntil(() -> elevatorSubsystem.isNearGoal()),
                coralSubsystem.trough().until(() -> (!coralSubsystem.outSensor())));
    }

    public Command dropCoral() {
        return coralSubsystem.runIntake(CORAL_OUTTAKE_SPEED).until(() -> (!coralSubsystem.outSensor()));
    }




    public Command autoElevDown() {
        return elevatorSubsystem.setHeight(ELEV_DOWN);
    }

    public Command autoAlignLeft() {
        return Commands.sequence(
                drivetrainSubsystem.applyRequest(() -> autoRobotDrive.withSpeeds(new ChassisSpeeds(0.0, 0.6, 0.0)))
                        .until(() -> !drivetrainSubsystem.autonSeesLeftSensor()),
                drivetrainSubsystem.autoStop()
        );
    }

    public Command autoAlignRight() {
        return Commands.sequence(
                drivetrainSubsystem.applyRequest(() -> autoRobotDrive.withSpeeds(new ChassisSpeeds(0.0, -0.75, 0.0)))
                        .until(() -> !drivetrainSubsystem.autonSeesRightSensor()), // 0.3
                drivetrainSubsystem.autoStop()
        );
    }

    public Command autoDriveForward() {
        return Commands.sequence(
                drivetrainSubsystem.applyRequest(() -> autoRobotDrive.withSpeeds(new ChassisSpeeds(0.6, 0.0, 0.0)))
                        .until(() -> (drivetrainSubsystem.seesLeftSensorClose() | drivetrainSubsystem.seesRightSensorClose())),
                drivetrainSubsystem.autoStop()
        // Commands.none()
        );
    }

    public Command autonBombsAway() {
        return Commands.sequence(

                elevatorSubsystem.setHeight(ELEV_BARGE),
                Commands.waitUntil(() -> (elevatorSubsystem.getElevatorEncoder() > 42.5))

        );
    }

    public Command autonDetonateFirst() {
        return Commands.sequence(

                algaeSubsystem.setVoltage(-1.5).withTimeout(0.05)

        );
    }

    public Command autonDetonateSecond() {
        return Commands.sequence(

                algaeSubsystem.setVoltage(5.0).until(() -> algaeSubsystem.getPivotEncoder() > 50)

        );
    }

    public Command autonDetonateThird() {
        return Commands.sequence(

                algaeSubsystem.setIntake(ALGAE_INTAKE_SLOW_SPEED),
                algaeSubsystem.setAngle(PIVOT_UP)
        );
    }

    public Command autonRegurgitateAlgaeFirst() {
        return Commands.sequence(
                algaeSubsystem.setIntake(ALGAE_INTAKE_SLOW_SPEED).withTimeout(0.6));
    }

    public Command autonRegurgitateAlgaeSecond() {
        return Commands.sequence(
                algaeSubsystem.stopIntake());
    }

    public Command autonTakeCover() {
        return Commands.sequence(
                elevatorSubsystem.setHeight(0.0),
                algaeSubsystem.setAngle(-90),
                algaeSubsystem.stopIntake()

        );

    }

}
