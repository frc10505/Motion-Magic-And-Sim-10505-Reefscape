/*
 * Copyright (C) 2025 Team 10505 All rights reserved. This work is
 * licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team10505.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.team10505.robot.subsystems.DrivetrainSubsystem;
import frc.team10505.robot.subsystems.ElevatorSubsystem;
import frc.team10505.robot.generated.TunerConstants;
import frc.team10505.robot.subsystems.AlgaeSubsystem;
import frc.team10505.robot.subsystems.CoralSubsystem;
import frc.team10505.robot.VisionStuff.Vision;
import frc.team10505.robot.Simulation.Simulation;
import static edu.wpi.first.units.Units.*;
import static frc.team10505.robot.Constants.*;

public class RobotContainer {

        private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

        /* Setting up bindings for necessary control of the swerve drive platform */
        private SwerveRequest.RobotCentric robotDrive = new SwerveRequest.RobotCentric()
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive
                                                                                 // motors

        private final Telemetry logger = new Telemetry(MaxSpeed);

        /* Controllers */
        private final CommandXboxController xboxController = new CommandXboxController(0);
        private final CommandXboxController xboxController2 = new CommandXboxController(1);

        private CommandJoystick joystick;
        private CommandJoystick joystick2;
        private CommandJoystick joystick3;
        private CommandJoystick joystick4;

        /* Subsystems */
        private final ElevatorSubsystem elevatorSubsys = new ElevatorSubsystem();
        private final AlgaeSubsystem algaeSubsys = new AlgaeSubsystem();
        private final CoralSubsystem coralSubsys;
        private final DrivetrainSubsystem drivetrainSubsys;
        private final Vision vision = new Vision();
        private Simulation simulation;

        /* Superstructure */
        private final Superstructure superStructure;
        /* Autonomous */
        private final SendableChooser<Command> svsuAutoChooser;

        private final SendableChooser<Double> polarityChooser = new SendableChooser<>();

        public RobotContainer() {
                if (Utils.isSimulation() || Utils.isReplay()) {
                        joystick = new CommandJoystick(0);
                        joystick2 = new CommandJoystick(1);
                        joystick3 = new CommandJoystick(2);
                        joystick4 = new CommandJoystick(3);

                        drivetrainSubsys = TunerConstants.createDrivetrain(joystick, vision);
                        coralSubsys = new CoralSubsystem(joystick);
                        simulation = new Simulation(algaeSubsys, coralSubsys, elevatorSubsys);

                        vision.reset();
                } else {
                        drivetrainSubsys = TunerConstants.createDrivetrain(vision);
                        coralSubsys = new CoralSubsystem();
                }

                superStructure = new Superstructure(coralSubsys, algaeSubsys, elevatorSubsys,
                                drivetrainSubsys);

                // commands registered to pathplanner
                NamedCommands.registerCommand("Test", Commands.print("auto command stuff is working"));

                NamedCommands.registerCommand("setElevToZero", elevatorSubsys.setHeight(ELEV_DOWN));
                NamedCommands.registerCommand("setElevToL2", elevatorSubsys.setHeight(ELEV_L2));
                NamedCommands.registerCommand("setElevTo24", elevatorSubsys.setHeight(ELEV_L3));
                NamedCommands.registerCommand("setElevTo48/5", elevatorSubsys.setHeight(ELEV_L4));

                NamedCommands.registerCommand("autoScoreCoralL4", superStructure.autoScoreCoralL4());
                NamedCommands.registerCommand("autoScoreCoralL2", superStructure.autoScoreCoralL2());
                NamedCommands.registerCommand("autoScoreCoralL1", superStructure.autoScoreCoralL1());

                NamedCommands.registerCommand("autoL4Bump", superStructure.autoL4Bump());

                NamedCommands.registerCommand("intakeCoral", superStructure.intakeCoral());
                NamedCommands.registerCommand("dropCoral", superStructure.dropCoral());

                NamedCommands.registerCommand("svsuAutoAlignLeft", superStructure.autoAlignLeft());
                NamedCommands.registerCommand("svsuAutoAlignRight", superStructure.autoAlignRight());

                NamedCommands.registerCommand("raisinPivot", algaeSubsys.setAngle(10));
                NamedCommands.registerCommand("setAlgaeIntake", (algaeSubsys.setIntake(0.5)));
                NamedCommands.registerCommand("stopAlgaeIntake", (algaeSubsys.stopIntake()));

                NamedCommands.registerCommand("autoBombsAway", superStructure.autonBombsAway());
                NamedCommands.registerCommand("autonDetonateFirst", superStructure.autonDetonateFirst());
                NamedCommands.registerCommand("autonDetonateSecond", superStructure.autonDetonateSecond());
                NamedCommands.registerCommand("autonDetonateThird", superStructure.autonDetonateThird());
                NamedCommands.registerCommand("autonRegurgitateAlgaeFirst",
                                superStructure.autonRegurgitateAlgaeFirst());
                NamedCommands.registerCommand("autonRegurgitateAlgaeSecond",
                                superStructure.autonRegurgitateAlgaeSecond());
                NamedCommands.registerCommand("autonTakeCover", superStructure.autonTakeCover());

                drivetrainSubsys.configPathplanner();

                svsuAutoChooser = AutoBuilder.buildAutoChooser();

                // Polarity chooser to be able to make the drive direction opposite
                SmartDashboard.putData("Polarity Chooser", polarityChooser);
                polarityChooser.setDefaultOption("Default", 1.0);
                polarityChooser.addOption("positive", 1.0);
                polarityChooser.addOption("Negative", -1.0);

                configDefaultCommands();
                configButtonBindings();
                configAutonomous();

        }

        /**
         * Function that is called in the constructor where we configure default
         * commands for the subsytems.
         */
        private void configDefaultCommands() {
                if (Utils.isSimulation() || Utils.isReplay()) {

                        drivetrainSubsys.setDefaultCommand(drivetrainSubsys.applyRequest(() -> drive
                                        .withVelocityX(-xboxController.getLeftX() * polarityChooser.getSelected()
                                                        * 0.8 * MaxSpeed) // Drive
                                        .withVelocityY(xboxController.getLeftY() * polarityChooser.getSelected() * // was
                                                                                                                   // negative
                                                        0.8 * MaxSpeed) // Drive left with negative X (left)
                                        .withRotationalRate(xboxController2.getLeftY() * 3.2 * MaxAngularRate))); // 2.5

                } else {
                        drivetrainSubsys.setDefaultCommand(drivetrainSubsys.applyRequest(() -> drive
                                        .withVelocityX(-xboxController.getLeftY() * polarityChooser.getSelected()
                                                        * 0.8 * MaxSpeed) // Drive
                                        .withVelocityY(-xboxController.getLeftX() * polarityChooser.getSelected() * // was
                                                                                                                    // negative
                                                        0.8 * MaxSpeed) // Drive left with negative X (left)
                                        .withRotationalRate(-xboxController.getRightX() * 3.2 * MaxAngularRate))); // 2.5
                }
        }

        /**
         * Function that is called in the constructor where we configure operator
         * interface button bindings.
         */
        private void configButtonBindings() {

                if (Utils.isSimulation() || Utils.isReplay()) {
                        // joystick.button(1).onTrue(superStructure.seekLeftIntakeStation());
                        joystick.button(1).onTrue(elevatorSubsys.setHeight(0));
                        joystick.button(2).onTrue(elevatorSubsys.setHeight(0.15));
                        joystick.button(3).onTrue(elevatorSubsys.setHeight(0.3));
                        joystick.button(4).onTrue(elevatorSubsys.setHeight(0.5));

                        joystick2.button(1).whileTrue(superStructure.intakeCoral());
                        joystick2.button(3).whileTrue(coralSubsys.runIntake(-CORAL_INTAKE_SPEED));
                        joystick2.button(4).whileTrue(coralSubsys.runIntake(CORAL_OUTTAKE_SPEED));

                        joystick3.button(1).onTrue(algaeSubsys.setAngle(ALGAE_PIVOT_DOWN));
                        joystick3.button(2).onTrue(algaeSubsys.setAngle(ALGAE_PIVOT_OUT));
                        joystick3.button(3).onTrue(algaeSubsys.setAngle(ALGAE_PIVOT_UP));
                        joystick3.button(4).whileTrue(algaeSubsys.setIntake(ALGAE_INTAKE_NORMAL_SPEED))
                                        .onFalse(algaeSubsys.stopIntake());

                } else {

                        // TODO check w comp code
                        // bindings for the xbox controller
                        xboxController.leftBumper().onTrue(algaeSubsys.setIntake(-ALGAE_INTAKE_NORMAL_SPEED))
                                        .onFalse(algaeSubsys.stopIntake());
                        xboxController.rightBumper().onTrue(algaeSubsys.setIntake(ALGAE_INTAKE_NORMAL_SPEED))
                                        .onFalse(superStructure.holdAlgae());
                        // xboxController.povDown().onTrue(superStructure.grabAlgae()).onFalse(superStructure.holdAlgae());

                        xboxController.leftTrigger()
                                        .whileTrue(drivetrainSubsys.applyRequest(() -> drive
                                                        .withVelocityX(-xboxController.getLeftY()
                                                                        * polarityChooser.getSelected()
                                                                        * 0.3 * MaxSpeed) // Drive
                                                        .withVelocityY(-xboxController.getLeftX()
                                                                        * polarityChooser.getSelected() *
                                                                        0.3 * MaxSpeed) // Drive left with negative X
                                                                                        // (left)
                                                        .withRotationalRate(
                                                                        -xboxController.getRightX() * 0.7
                                                                                        * MaxAngularRate)));
                        xboxController.rightTrigger()
                                        .whileTrue(drivetrainSubsys.applyRequest(() -> drive
                                                        .withVelocityX(-xboxController.getLeftY()
                                                                        * polarityChooser.getSelected()
                                                                        * 1.2 * MaxSpeed) // Drive
                                                        .withVelocityY(-xboxController.getLeftX()
                                                                        * polarityChooser.getSelected() *
                                                                        1.2 * MaxSpeed) // Drive left with negative X
                                                                                        // (left)
                                                        .withRotationalRate(
                                                                        -xboxController.getRightX() * 3.2
                                                                                        * MaxAngularRate)));

                        xboxController.a().onTrue(algaeSubsys.setAngle(-18));
                        xboxController.b().onTrue(algaeSubsys.setIntake(0.2)).onFalse(algaeSubsys.stopIntake());
                        xboxController.x().onTrue(algaeSubsys.setAngle(-90));
                        xboxController.y().onTrue(algaeSubsys.setAngle(10));

                        xboxController.start().onTrue(resetGyro());
                        xboxController.back().onTrue(resetGyro180());

                        xboxController.povUp().whileTrue(
                                        drivetrainSubsys.applyRequest(() -> robotDrive.withVelocityX(-0.4)
                                                        .withVelocityY(0.0)
                                                        .withRotationalRate(0.0)))
                                        .onFalse(drivetrainSubsys.autoStop());// BAD BAD BAD (maybe not ig)

                        xboxController.povDown().whileTrue(
                                        drivetrainSubsys.applyRequest(() -> robotDrive.withVelocityX(0.4)
                                                        .withVelocityY(0.0)
                                                        .withRotationalRate(0.0)))
                                        .onFalse(drivetrainSubsys.autoStop()); // BAD BAD BAD (maybe not ig)

                        xboxController.povLeft().whileTrue(
                                        drivetrainSubsys.applyRequest(() -> robotDrive.withVelocityX(0.0)
                                                        .withVelocityY(0.6)// *
                                                        .withRotationalRate(0.0))
                                                        .until(() -> !drivetrainSubsys.seesLeftSensor()));

                        xboxController.povRight().whileTrue(
                                        // rumblyRightAlign()
                                        drivetrainSubsys.applyRequest(() -> robotDrive.withVelocityX(0.0)
                                                        .withVelocityY(-0.6)// 75)// *
                                                                            // rightAutoAlignSpeedMULTIPLIER.getSelected())
                                                        .withRotationalRate(0.0))
                                                        .until(() -> !drivetrainSubsys.seesRightSensor()));

                        // operator bindings
                        xboxController2.povUp().onTrue(superStructure.outputTopCoral());
                        xboxController2.povDown().onTrue(superStructure.intakeCoral());// .onFalse(coralSubsys.stop());
                        xboxController2.povLeft().whileTrue(superStructure.outputCoral());
                        xboxController2.povRight().whileTrue(superStructure.outputCoralTrough());

                        xboxController2.x().onTrue(elevatorSubsys.setHeight(ELEV_DOWN));
                        xboxController2.a().onTrue(elevatorSubsys.setHeight(ELEV_L2));
                        xboxController2.b().onTrue(elevatorSubsys.setHeight(ELEV_L3));
                        xboxController2.y().onTrue(elevatorSubsys.setHeight(ELEV_L4));
                        xboxController2.rightBumper().onTrue(superStructure.manualL4Bump());

                        // nootont
                        xboxController2.rightTrigger().onTrue(superStructure.bombsAway());
                        xboxController2.leftBumper().onTrue(superStructure.detonate());
                        xboxController2.leftTrigger().onTrue(superStructure.takeCover());

                        // automoatically added from the CTRE generated swerve drive
                        // probably is important?
                        drivetrainSubsys.registerTelemetry(logger::telemeterize);
                }
        }

        private Command resetGyro() {
                return Commands.runOnce(() -> {
                        drivetrainSubsys.getPigeon2().reset();

                });
        }

        private Command resetGyro180() {
                return Commands.runOnce(() -> {
                        drivetrainSubsys.getPigeon2().setYaw(180);
                });
        }

        private Command resetPose() {
                return Commands.runOnce(() -> {
                        drivetrainSubsys.resetPose(new Pose2d(0.0, 0.0, new Rotation2d(0.0)));
                });
        }

        // methods that allow us to select and use our auton selected in our dashboards
        private void configAutonomous() {
                SmartDashboard.putData(svsuAutoChooser);// autoChooser);
        }

        public Command getAutonomousCommand() {
                return svsuAutoChooser.getSelected();// autoChooser.getSelected();
        }

        /**Puts the estimated robot pose to smartdashboard(x, y, rot and gyro rot) */
        public void updatePoseValues() {
                SmartDashboard.putNumber("estimated drive pose x", drivetrainSubsys.getState().Pose.getX());
                SmartDashboard.putNumber("estimated drive pose y", drivetrainSubsys.getState().Pose.getY());
                SmartDashboard.putNumber("estimated drive pose rotation",
                                drivetrainSubsys.getState().Pose.getRotation().getDegrees());
                SmartDashboard.putNumber("Gyro rot",
                                drivetrainSubsys.getPigeon2().getRotation2d().getDegrees());
        }

        /**adds vision measurements to the drivetrain, updates the field viz and updates dashboard values from the cams */
        public void updateVision() {
                try {
                        vision.frontCamera.getEstimatedPose().ifPresent(est -> {
                                drivetrainSubsys.addVisionMeasurement(est.estimatedPose.toPose2d(),
                                                est.timestampSeconds);
                        });

                } catch (Exception e) {
                        SmartDashboard.putString("Failures", "Front cam pose estimator failed");
                }
                vision.updateViz(drivetrainSubsys.getState().Pose);
                vision.updateDashboard();
        }

        public void updateBatterySim() {
                RoboRioSim.setVInCurrent(BatterySim.calculateDefaultBatteryLoadedVoltage(
                                simulation.getMechanismCurrentDrawAms()));
                SmartDashboard.putNumber("SIM RobotRio v in current", RoboRioSim.getVInCurrent());
                SmartDashboard.putNumber("SIM RobotRio brownout voltage", RoboRioSim.getBrownoutVoltage());
                SmartDashboard.putNumber("SIM RobotRio user current 6v", RoboRioSim.getUserCurrent6V());


        }

}
