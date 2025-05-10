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
import static frc.team10505.robot.Constants.*;

public class Superstructure {
    /*variables */
    private CoralSubsystem coralSubsys;
    private AlgaeSubsystem algaeSubsys;
    private ElevatorSubsystem elevatorSubsys;
    private DrivetrainSubsystem drivetrainSubsys;

    private final SwerveRequest.ApplyRobotSpeeds autoRobotDrive = new SwerveRequest.ApplyRobotSpeeds();

    /*Constructor */
    public Superstructure(CoralSubsystem coralSubsystem, AlgaeSubsystem algaeSubsystem, ElevatorSubsystem elevatorSubsystem,
            DrivetrainSubsystem drivetrainSubsystem) {
        this.coralSubsys = coralSubsystem;
        this.algaeSubsys = algaeSubsystem;
        this.elevatorSubsys = elevatorSubsystem;
        this.drivetrainSubsys = drivetrainSubsystem;
    }

    public Command intakeCoral() {
        return coralSubsys.slowEndIntake(CORAL_INTAKE_SPEED).until(() -> (coralSubsys.outSensor() && coralSubsys.inSensor()));
    }

    public Command outputCoral() {
        return Commands.sequence(
                coralSubsys.runIntake(CORAL_OUTTAKE_SPEED).until(() -> (!coralSubsys.outSensor())),
                elevatorSubsys.setHeight(ELEV_DOWN));
    }

    public Command outputCoralTrough() {
        return coralSubsys.trough().until(() -> (!coralSubsys.outSensor()));    
    }

    public Command outputTopCoral() {
        return Commands.sequence(
                coralSubsys.runIntake(CORAL_OUTTAKE_TOP_SPEED),
                Commands.waitSeconds(0.5),
                elevatorSubsys.setHeight(ELEV_L4_BUMP),
                Commands.waitUntil(() -> (elevatorSubsys.isAbove(52.0))), 
                Commands.waitSeconds(0.2),
                coralSubsys.setIntake(0),
                elevatorSubsys.setHeight(ELEV_DOWN));
    }

    public Command bombsAway() {
        return Commands.sequence(
                elevatorSubsys.setHeight(ELEV_BARGE),
                Commands.waitUntil(() -> (elevatorSubsys.getElevatorEncoder() > 42.5))
        );
    }

    public Command detonate() {
        return Commands.sequence(

                algaeSubsys.setVoltage(-1.5).withTimeout(0.05),
                algaeSubsys.setVoltage(5.0).until(() -> algaeSubsys.getPivotEncoder() > 50),
                algaeSubsys.setIntake(0.3),
                algaeSubsys.setAngle(ALGAE_PIVOT_UP)

        );
    }

    // public Command regurgitateAlgae(){
    // return Commands.sequence(
    // algaeSubsys.intakeForwardSlower().withTimeout(0.6),
    // algaeSubsys.intakeStop()
    // );
    // }

    public Command takeCover() {
        return Commands.sequence(
                elevatorSubsys.setHeight(0.0),
                algaeSubsys.setAngle(-90),
                algaeSubsys.stopIntake()

        );

    }

    // public Command grabAlgae() {
    // return Commands.sequence(
    // algaeSubsys.intakeReverse(),
    // algaeSubsys.coastPivot());
    // }

    public Command holdAlgae() {
        return Commands.sequence(
                algaeSubsys.stopIntake(),
                algaeSubsys.setAngle(-13));
    }

    public Command manualL4Bump() {
        return Commands.sequence(
                elevatorSubsys.setHeight(55.0), // 54.5 //54 BADISH
                // Commands.waitUntil(() -> elevatorSubsys.isNearGoal()),
                Commands.waitUntil(() -> (elevatorSubsys.isAbove(52.0))),
                Commands.waitSeconds(0.2),
                coralSubsys.setIntake(0),
                elevatorSubsys.setHeight(0.0));
    }

    // public Command seekLeftIntakeStation(){
    //     return parallel(
    //         coralSubsys.runIntake(CORAL_INTAKE_SPEED).unless(() -> coralSubsys.inSensor()),
    //         drivetrainSubsys.goToLeftStation()
    //     );
    // }

    // COMMANDS FOR AUTONS
    public Command autoIntakeCoral() {
        return coralSubsys.slowEndIntake(CORAL_INTAKE_SPEED).until(() -> (coralSubsys.outSensor() && coralSubsys.inSensor()));
    }

    public Command autoOutputCoral() {
        return coralSubsys.runIntake(0.25).until(() -> (!coralSubsys.outSensor()));
    }

    public Command autoOutputCoralTrough() {
        return coralSubsys.trough().until(() -> (!coralSubsys.outSensor()));
    }

    public Command autoOutputTopCoral() {
        return coralSubsys.runIntake(0.2).until(() -> (!coralSubsys.outSensor()));
    }

    public Command autoHoldAlgae() {
        return algaeSubsys.stopIntake();
    }

    public Command autoScoreCoralL4() {
        return Commands.sequence(
                elevatorSubsys.setHeight(ELEV_L4), 
                Commands.waitUntil(() -> elevatorSubsys.isNearGoal()),
                coralSubsys.setIntake(CORAL_INTAKE_SPEED),//TODO figure out what the gaf is this .37? 
                //NOTE are we stupid? probably....... :(
                //other auto scores use .25,  and i think teleop l4 scoring uses 0.05
                Commands.race(
                        Commands.waitUntil(() -> !coralSubsys.outSensor()), 
                        Commands.waitSeconds(1.2)));
    }

    public Command autoL4Bump() {
        return Commands.sequence(
                elevatorSubsys.setHeight(ELEV_L4_BUMP), 
                Commands.waitUntil(() -> (elevatorSubsys.isAbove(52.0))),
                Commands.waitSeconds(0.2),
                coralSubsys.setIntake(0));
    }

    public Command autoScoreCoralL2() {
        return Commands.sequence(
                elevatorSubsys.setHeight(ELEV_L2),
                Commands.waitUntil(() -> elevatorSubsys.isNearGoal()),
                coralSubsys.setIntake(CORAL_OUTTAKE_SPEED),
                Commands.race(
                        Commands.waitUntil(() -> (!coralSubsys.outSensor())),
                        Commands.waitSeconds(2.5)),
                coralSubsys.setIntake(0));
    }

    public Command autoScoreCoralL1() {
        return Commands.sequence(
                elevatorSubsys.setHeight(ELEV_DOWN),
                Commands.waitUntil(() -> elevatorSubsys.isNearGoal()),
                coralSubsys.trough().until(() -> (!coralSubsys.outSensor())));
    }

    public Command dropCoral() {
        return coralSubsys.runIntake(CORAL_OUTTAKE_SPEED).until(() -> (!coralSubsys.outSensor()));
    }

    public Command autoAlignLeft() {
        return Commands.sequence(
                drivetrainSubsys.applyRequest(() -> autoRobotDrive.withSpeeds(new ChassisSpeeds(0.0, 0.6, 0.0)))
                        .until(() -> !drivetrainSubsys.autonSeesLeftSensor()),
                drivetrainSubsys.autoStop()
        );
    }

    public Command autoAlignRight() {
        return Commands.sequence(
                drivetrainSubsys.applyRequest(() -> autoRobotDrive.withSpeeds(new ChassisSpeeds(0.0, -0.75, 0.0)))
                        .until(() -> !drivetrainSubsys.autonSeesRightSensor()), // 0.3
                drivetrainSubsys.autoStop()
        );
    }

    public Command autoDriveForward() {
        return Commands.sequence(
                drivetrainSubsys.applyRequest(() -> autoRobotDrive.withSpeeds(new ChassisSpeeds(0.6, 0.0, 0.0)))
                        .until(() -> (drivetrainSubsys.seesLeftSensorClose() | drivetrainSubsys.seesRightSensorClose())),
                drivetrainSubsys.autoStop()
        // Commands.none()
        );
    }

    public Command autonBombsAway() {
        return Commands.sequence(

                elevatorSubsys.setHeight(ELEV_BARGE),
                Commands.waitUntil(() -> (elevatorSubsys.getElevatorEncoder() > 42.5))

        );
    }

    public Command autonDetonateFirst() {
        return Commands.sequence(

                algaeSubsys.setVoltage(-1.5).withTimeout(0.05)

        );
    }

    public Command autonDetonateSecond() {
        return Commands.sequence(

                algaeSubsys.setVoltage(5.0).until(() -> algaeSubsys.getPivotEncoder() > 50)

        );
    }

    public Command autonDetonateThird() {
        return Commands.sequence(

                algaeSubsys.setIntake(ALGAE_INTAKE_SLOW_SPEED),
                algaeSubsys.setAngle(ALGAE_PIVOT_UP)
        );
    }

    public Command autonRegurgitateAlgaeFirst() {
        return Commands.sequence(
                algaeSubsys.setIntake(ALGAE_INTAKE_SLOW_SPEED).withTimeout(0.6));
    }

    public Command autonRegurgitateAlgaeSecond() {
        return Commands.sequence(
                algaeSubsys.stopIntake());
    }

    public Command autonTakeCover() {
        return Commands.sequence(
                elevatorSubsys.setHeight(0.0),
                algaeSubsys.setAngle(-90),
                algaeSubsys.stopIntake()

        );

    }

}
