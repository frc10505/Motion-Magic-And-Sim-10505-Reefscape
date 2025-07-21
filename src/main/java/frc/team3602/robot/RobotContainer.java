package frc.team3602.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.team3602.robot.subsystems.drive.DrivetrainSubsystem;
import frc.team3602.robot.subsystems.drive.generated.TunerConstants;

public class RobotContainer {
    /*Controllers */
    private CommandXboxController xbox;
    private CommandXboxController xbox2;
    private CommandJoystick joystick;
    private CommandJoystick joystick2;

    private DrivetrainSubsystem driveSubsys = TunerConstants.createDrivetrain();
    private Vision vision = new Vision();

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive

    private StructPublisher<Pose2d> posePublisher = NetworkTableInstance.getDefault().getStructTopic("Drivetrain Pose", Pose2d.struct).publish();
         
    public RobotContainer(){
        vision.reset();
        if(RobotBase.isSimulation()){
            simConfigDefaultCommands();
            xbox = new CommandXboxController(0);
            xbox2 = new CommandXboxController(1);
        } 
    }


    private void simConfigDefaultCommands(){
        driveSubsys.setDefaultCommand(driveSubsys.applyRequest(() -> drive
                                        .withVelocityX(-xbox.getLeftX() 
                                                        * 0.8 * MaxSpeed) // Drive
                                        .withVelocityY(xbox.getLeftY() *// was
                                                                                                                   // negative
                                                        0.8 * MaxSpeed) // Drive left with negative X (left)
                                        .withRotationalRate(xbox2.getLeftY() * 3.2 * MaxAngularRate))); // 2.5

    }

    public void updateVisionSim(){
        vision.updateResults();
        vision.updateViz(driveSubsys.getState().Pose);
        vision.putTargetValues(vision.frontCam);
       // vision.putTargetValues(vision.backCam);
        vision.putPEValues(vision.getFrontCamEstimatedPose(), vision.frontCam);
      //  vision.putPEValues(vision.getBackCamEstimatedPose(), vision.backCam);
    }

    public void addVisionMeas(){
        vision.updatePoseEstimations();
        driveSubsys.addVisionMeasurement(vision.frontCamEstimatedPose, vision.frontCamLastPoseTS);
        driveSubsys.addVisionMeasurement(vision.backCamEstimatedPose, vision.backCamLastPoseTS);
        posePublisher.set(driveSubsys.getState().Pose);
    }
}
