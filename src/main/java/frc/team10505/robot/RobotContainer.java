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

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.team10505.robot.generated.TunerConstants;
import frc.team10505.robot.subsystems.DrivetrainSubsystem;
import frc.team10505.robot.subsystems.ElevatorSubsystem;
import frc.team10505.robot.subsystems.AlgaeSubsystem;
import frc.team10505.robot.subsystems.CoralSubsystem;
import static edu.wpi.first.units.Units.*;
import static frc.team10505.robot.Constants.OperatorInterfaceConstants.*;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.photonvision.simulation.VisionSystemSim;

public class RobotContainer {

        private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
                                                                                      // speed
        private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // was .75 3/4 of a rotation
                                                                                          // per
                                                                                          // second
                                                                                          // max angular velocity

        /* Setting up bindings for necessary control of the swerve drive platform */
        private SwerveRequest.RobotCentric robotDrive = new SwerveRequest.RobotCentric()
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive
                                                                                 // motors

        private final Telemetry logger = new Telemetry(MaxSpeed);

        /* Operator interfaces */
        private final CommandXboxController xboxController = new CommandXboxController(0);
        private final CommandXboxController xboxController2 = new CommandXboxController(kControlPanelPort);

        private final CommandJoystick joystick = new CommandJoystick(0);
        private final CommandJoystick joystick2 = new CommandJoystick(1);

        /* Subsystems */
        private final DrivetrainSubsystem drivetrainSubsys;
        private final AlgaeSubsystem algaeSubsys = new AlgaeSubsystem();
        private final CoralSubsystem coralSubsys;
        private final ElevatorSubsystem elevatorSubsys = new ElevatorSubsystem();
        private final Vision vision = new Vision();

        /* Superstructure */
        private final Superstructure superStructure;
        /* Autonomous */
        // private final SendableChooser<Command> autoChooser;
        private final SendableChooser<Command> svsuAutoChooser;

        private final SendableChooser<Double> polarityChooser = new SendableChooser<>();

        public RobotContainer() {
                if (Utils.isSimulation()) {
                        drivetrainSubsys = TunerConstants.createDrivetrain(joystick);
                        coralSubsys = new CoralSubsystem(joystick);

                } else {
                        drivetrainSubsys = TunerConstants.createDrivetrain();
                        coralSubsys = new CoralSubsystem();
                }

                superStructure = new Superstructure(coralSubsys, algaeSubsys, elevatorSubsys,
                                drivetrainSubsys);

                NamedCommands.registerCommand("Test", Commands.print("auto command stuff is working"));

                NamedCommands.registerCommand("setElevToZero", elevatorSubsys.setHeight(0.0));
                NamedCommands.registerCommand("setElevToL2", elevatorSubsys.setHeight(8.0));
                NamedCommands.registerCommand("setElevTo24", elevatorSubsys.setHeight(24.0));
                NamedCommands.registerCommand("setElevTo48/5", elevatorSubsys.setHeight(48.5));

                NamedCommands.registerCommand("intakeCoral", superStructure.intakeCoral());
                NamedCommands.registerCommand("dropCoral", superStructure.dropCoral());

                NamedCommands.registerCommand("autoScoreCoralL3", superStructure.autoScoreCoralL3());
                NamedCommands.registerCommand("autoScoreCoralL4", superStructure.autoScoreCoralL4());
                NamedCommands.registerCommand("autoScoreCoralL2", superStructure.autoScoreCoralL2());
                NamedCommands.registerCommand("autoScoreCoralL1", superStructure.autoScoreCoralL1());

                NamedCommands.registerCommand("autoElevDown", superStructure.autoElevDown());
                NamedCommands.registerCommand("autoL4Bump", superStructure.autoL4Bump());

                NamedCommands.registerCommand("autoAlignLeft", superStructure.autoAlignLeft());
                NamedCommands.registerCommand("autoAlignRight",
                                superStructure.autoAlignRight());
                NamedCommands.registerCommand("autoDriveForward", superStructure.autoDriveForward());

                NamedCommands.registerCommand("raisinPivot", algaeSubsys.setAngle(10));
                NamedCommands.registerCommand("setAlgaeIntake", (algaeSubsys.intakeForward()));
                NamedCommands.registerCommand("stopAlgaeIntake", (algaeSubsys.intakeStop()));

                NamedCommands.registerCommand("svsuAutoAlignLeft", superStructure.autoAlignLeft());
                NamedCommands.registerCommand("svsuAutoAlignRight",
                                superStructure.autoAlignRight());

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

                // autoChooser = AutoBuilder.buildAutoChooser();
                svsuAutoChooser = AutoBuilder.buildAutoChooser();

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
                if (Utils.isSimulation()) {

                        drivetrainSubsys.setDefaultCommand(drivetrainSubsys.applyRequest(() -> drive
                                        .withVelocityX(-xboxController.getLeftX() * polarityChooser.getSelected()
                                                        * 0.8 * MaxSpeed) // Drive
                                        .withVelocityY(xboxController.getLeftY() * polarityChooser.getSelected() * // was
                                                                                                                   // negative
                                                        0.8 * MaxSpeed) // Drive left with negative X (left)
                                        .withRotationalRate(xboxController2.getLeftY() * 3.2 * MaxAngularRate))); // 2.5

                }

                else {
                        drivetrainSubsys.setDefaultCommand(drivetrainSubsys.applyRequest(() -> drive
                                        .withVelocityX(-xboxController.getLeftY() * polarityChooser.getSelected()
                                                        * 0.8 * MaxSpeed) // Drive
                                        .withVelocityY(-xboxController.getLeftX() * polarityChooser.getSelected() * // was
                                                                                                                    // negative
                                                        0.8 * MaxSpeed) // Drive left with negative X (left)
                                        .withRotationalRate(-xboxController.getRightX() * 3.2 * MaxAngularRate))); // 2.5
                }

                /* OLD joystick bindings */
                // drivetrainSubsys.applyRequest(
                // () -> drive.withVelocityX(-joystick.getY() * 0.8 *
                // polarityChooser.getSelected() * MaxSpeed) //0.6// Drive

                // .withVelocityY(-joystick.getX() * 0.8 * polarityChooser.getSelected() *
                // MaxSpeed) // Drive
                // // left
                // // with
                // // negative
                // // X
                // // (left)
                // //.withRotationalRate(-joystick.getTwist() * 1.6 * MaxAngularRate) // Drive
                // // counterclockwise
                // // with negative X
                // // (left)
                // ));

        }

        /**
         * Function that is called in the constructor where we configure operator
         * interface button bindings.
         */
        private void configButtonBindings() {

                if (Utils.isSimulation()) {
                        joystick.button(1).onTrue(elevatorSubsys.setHeight(0));// coralSubsys.slowEndIntake());//
                        joystick.button(2).onTrue(elevatorSubsys.setHeight(1));// coralSubsys.trough().until (() ->
                                                                               // !coralSubsys.outSensor()));
                        joystick.button(3).onTrue(elevatorSubsys.setHeight(2));
                        joystick.button(4).onTrue(elevatorSubsys.setHeight(3));

                        // joystick.button(1).onTrue(algaeSubsys.setAngle(-90));
                        // joystick.button(2).onTrue(algaeSubsys.setAngle(-30));
                        // joystick.button(3).onTrue(algaeSubsys.setAngle(0));
                        // joystick.button(4).onTrue(algaeSubsys.setAngle(90));

                        joystick2.button(3).whileTrue(coralSubsys.runIntake(-1));
                        joystick2.button(4).whileTrue(coralSubsys.runIntake(3));
                        joystick2.button(1).whileTrue(coralSubsys.runIntake(5));

                } else {
                        // bindings for the xbox controller
                        xboxController.leftBumper().onTrue(algaeSubsys.intakeReverse())
                                        .onFalse(algaeSubsys.intakeStop());
                        xboxController.rightBumper().onTrue(algaeSubsys.intakeForward())
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
                        xboxController.b().onTrue(algaeSubsys.intakeForwardSlowest()).onFalse(algaeSubsys.intakeStop());
                        // setLights());
                        // resetPose());// onTrue(algaeSubsys.stopPivot());
                        xboxController.x().onTrue(algaeSubsys.setAngle(-90));
                        xboxController.y().onTrue(algaeSubsys.setAngle(10)); // 5

                        xboxController.start().onTrue(resetGyro());
                        xboxController.back().onTrue(resetGyro180());// check which bindings

                        // xboxController.l().onTrue(superStructure.grabAlgae()).onFalse(superStructure.holdAlgae());

                        xboxController.povUp().whileTrue(
                                        drivetrainSubsys.applyRequest(() -> robotDrive.withVelocityX(-0.4)
                                                        .withVelocityY(0.0)
                                                        .withRotationalRate(0.0)))
                                        .onFalse(drivetrainSubsys.stop());

                        xboxController.povDown().whileTrue(
                                        drivetrainSubsys.applyRequest(() -> robotDrive.withVelocityX(0.4)
                                                        .withVelocityY(0.0)
                                                        .withRotationalRate(0.0)))
                                        .onFalse(drivetrainSubsys.stop());

                        xboxController.povLeft().whileTrue(
                                        drivetrainSubsys.applyRequest(() -> robotDrive.withVelocityX(0.0)
                                                        .withVelocityY(0.6)// *
                                                                           // leftAutoAlignSpeedMULTIPLIER.getSelected())
                                                        .withRotationalRate(0.0))
                                                        .until(() -> !drivetrainSubsys.seesLeftSensor())
                        // .andThen(
                        // setLeftRumbles())
                        );

                        xboxController.povRight().whileTrue(
                                        // rumblyRightAlign()
                                        drivetrainSubsys.applyRequest(() -> robotDrive.withVelocityX(0.0)
                                                        .withVelocityY(-0.6)// 75)// *
                                                                            // rightAutoAlignSpeedMULTIPLIER.getSelected())
                                                        .withRotationalRate(0.0))
                                                        .until(() -> !drivetrainSubsys.seesRightSensor())
                        // .andThen(
                        // setRightRumbles())
                        );

                        // NEW joystick bindings

                        // joystick.trigger()
                        // .whileTrue(drivetrainSubsys.applyRequest(
                        // () -> drive.withVelocityX(-joystick.getY() * 0.2 *
                        // polarityChooser.getSelected() * MaxSpeed) // Drive
                        // .withVelocityY(-joystick.getX() * 0.2 * polarityChooser.getSelected() *
                        // MaxSpeed) // Drive
                        // // left
                        // // with
                        // // negative
                        // // X
                        // // (left)
                        // .withRotationalRate(-joystick.getTwist() * 0.5 * MaxAngularRate) // Drive
                        // // counterclockwise
                        // // with negative X
                        // // (left)
                        // ));

                        // joystick.button(12).onTrue(algaeSubsys.setAngle(-22));// -18
                        // joystick.button(9).onTrue(algaeSubsys.setAngle(90));//-90
                        // joystick.button(10).onTrue(algaeSubsys.setAngle(5));

                        // joystick.button(3).onTrue(algaeSubsys.intakeReverse()).onFalse(algaeSubsys.intakeStop());
                        // joystick.button(5).onTrue(algaeSubsys.intakeForward()).onFalse(superStructure.holdAlgae());

                        // joystick.povLeft().whileTrue(drivetrainSubsys.alignLeft());

                        // operator bindings
                        xboxController2.povUp().onTrue(superStructure.outputTopCoral());
                        xboxController2.povDown().onTrue(superStructure.intakeCoral());// .onFalse(coralSubsys.stop());
                        xboxController2.povLeft().whileTrue(superStructure.outputCoral());
                        xboxController2.povRight().whileTrue(superStructure.outputCoralTrough());

                        xboxController2.a().onTrue(elevatorSubsys.setHeight(8.0));
                        xboxController2.b().onTrue(elevatorSubsys.setHeight(23.5));// maybe change?//$$24
                        xboxController2.x().onTrue(elevatorSubsys.setHeight(0.0));
                        xboxController2.y().onTrue(elevatorSubsys.setHeight(48.5));
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

        public void updateDriveSensors() {
                SmartDashboard.putBoolean("left drive sensor", drivetrainSubsys.seesLeftSensor());
                SmartDashboard.putBoolean("right drive sensor", drivetrainSubsys.seesRightSensor());
        }

        // called periodically in robot.java, updates all our pose estimation stuff
        public void updatePose() {
                // puts the drivetrain pose on our dashboards
                SmartDashboard.putNumber("estimated drive pose x", drivetrainSubsys.getState().Pose.getX());
                SmartDashboard.putNumber("estimated drive pose y", drivetrainSubsys.getState().Pose.getY());
                SmartDashboard.putNumber("estimated drive pose rotation",
                                drivetrainSubsys.getState().Pose.getRotation().getDegrees());
                SmartDashboard.putNumber("Gyro rot",
                                drivetrainSubsys.getPigeon2().getRotation2d().getDegrees());

        }

        private VisionSystemSim vizSim = new VisionSystemSim("Reefscape Sim");

        public void vizSimInit() {
                vizSim.addAprilTags(AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded));
        }

        public void updateVizSim() {
                vizSim.update(drivetrainSubsys.getState().Pose);
                vizSim.getDebugField();
        }

        public void cameraFeedInit() {
                var m_visionThread = new Thread(
                                () -> {
                                        // Get the UsbCamera from CameraServer
                                        UsbCamera camera = CameraServer.startAutomaticCapture();
                                        // Set the resolution
                                        camera.setResolution(280, 480);// 680

                                        camera.setFPS(20);
                                        // Get a CvSink. This will capture Mats from the camera
                                        CvSink cvSink = CameraServer.getVideo();
                                        // Setup a CvSource. This will send images back to the Dashboard
                                        CvSource outputStream = CameraServer.putVideo("Rectangle", 640, 480);

                                        // Mats are very memory expensive. Lets reuse this Mat.
                                        Mat mat = new Mat();

                                        // This cannot be 'true'. The program will never exit if it is. This
                                        // lets the robot stop this thread when restarting robot code or
                                        // deploying.
                                        while (!Thread.interrupted()) {
                                                // Tell the CvSink to grab a frame from the camera and put it
                                                // in the source mat. If there is an error notify the output.
                                                if (cvSink.grabFrame(mat) == 0) {
                                                        // Send the output the error.
                                                        outputStream.notifyError(cvSink.getError());
                                                        // skip the rest of the current iteration
                                                        continue;
                                                }
                                                // Put a rectangle on the image
                                                Imgproc.rectangle(
                                                                mat, new Point(100, 100), new Point(400, 400),
                                                                new Scalar(255, 255, 255), 5);
                                                // Give the output stream a new image to display
                                                outputStream.putFrame(mat);
                                        }
                                });
                m_visionThread.setDaemon(true);
                m_visionThread.start();
        }

        
        public void updateVisionPose(){

        var mostRecentReefCamPose = new Pose2d();

        // if the camera has a tag in sight, it will calculate a pose and add to the
        // drivetrain
        // if it fails, it will print the message

        try {

            var visionEst = vision.simGetReefCamEstimatedPose();
            visionEst.ifPresent(est -> {
                drivetrainSubsys.addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds);
                SmartDashboard.putNumber("reef cam pose x", est.estimatedPose.toPose2d().getX());
                SmartDashboard.putNumber("reef cam pose y", est.estimatedPose.toPose2d().getY());
                SmartDashboard.putNumber("reef cam pose rot", est.estimatedPose.toPose2d().getRotation().getDegrees());
            });

           

        } catch (Exception e) {
            Commands.print("reef cam pose failed");
        }
        }

}
