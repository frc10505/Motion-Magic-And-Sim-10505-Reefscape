package frc.team10505.robot;



import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team10505.robot.subsystems.AlgaeSubsystem;
import frc.team10505.robot.subsystems.CoralSubsystem;
import frc.team10505.robot.subsystems.ElevatorSubsystem;
import org.littletonrobotics.junction.Logger;


public class Simulation extends SubsystemBase {
    private AlgaeSubsystem algaeSubsys;
    private CoralSubsystem coralSubsys;
    private ElevatorSubsystem elevSubsys;

    //Colors
    private final Color8Bit red = new Color8Bit(Color.kFirstRed);
    private final Color8Bit green = new Color8Bit(Color.kGreen);

    //3d sim publishers
    StructPublisher<Pose3d> algaePivotPublisher = NetworkTableInstance.getDefault().getStructTopic("Algae Pivot Pos", Pose3d.struct).publish();
    StructPublisher<Pose3d> algaeIntakePublisher = NetworkTableInstance.getDefault().getStructTopic("Algae Intake Pos", Pose3d.struct).publish();
    StructPublisher<Pose3d> coralLeftWheelsPublisher = NetworkTableInstance.getDefault().getStructTopic("Coral Left Wheels Pos", Pose3d.struct).publish();
    StructPublisher<Pose3d> coralRightWheelsPublisher = NetworkTableInstance.getDefault().getStructTopic("Coral Right Wheels Pos", Pose3d.struct).publish();
    StructPublisher<Pose3d> elevatorPublisher = NetworkTableInstance.getDefault().getStructTopic("Elevator Pos", Pose3d.struct).publish();

    //2d Elev sim Viz
    private final Mechanism2d elevMech = new Mechanism2d(1.5, 1.5);
    private final MechanismRoot2d elevRoot = elevMech.getRoot("elevRoot", 0.75, 0.1);
    private final MechanismLigament2d elevViz;

    //2d coral sim viz
    private final FlywheelSim intakeLeftSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), 0.003, 5), DCMotor.getNEO(1));

    private final FlywheelSim intakeRightSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), 0.003, 5), DCMotor.getNEO(1));

    private final Mechanism2d coralIntakeMech = new Mechanism2d(3, 1.2);

    private final MechanismRoot2d leftSimRoot = coralIntakeMech.getRoot("leftRoot", .6, 0.6);
    private final MechanismRoot2d rightSimRoot = coralIntakeMech.getRoot("rightRoot", 2.4, 0.6);

    private final MechanismRoot2d inSensorRoot = coralIntakeMech.getRoot("inSensorRoot", 1.5, 1);
    private final MechanismRoot2d outSensorRoot = coralIntakeMech.getRoot("outSensorRoot", 1.5, 0.2);

    public final MechanismLigament2d leftIntakeViz = leftSimRoot
            .append(new MechanismLigament2d("leftIntakeLigament", 0.4, 000));
    public final MechanismLigament2d rightIntakeViz = rightSimRoot
            .append(new MechanismLigament2d("rightIntakeLigament", 0.4, 180));

    private final MechanismLigament2d inSensorViz = inSensorRoot
            .append(new MechanismLigament2d("inSensorLigament", 0.15, 90, 40, red));
    private final MechanismLigament2d outSensorViz = outSensorRoot
            .append(new MechanismLigament2d("outSensorLigament", 0.15, -90, 40, red));

    //2d Algae viz sim
    private final Mechanism2d algaeMech = new Mechanism2d(1.5, 1.5);
    private final MechanismRoot2d pivotRoot = algaeMech.getRoot("pivotRoot", 0.75, 0.75);
    public final MechanismLigament2d pivotViz;
    private final MechanismRoot2d intakeRoot = algaeMech.getRoot("intakeRoot", 1.31, 0.75);
    public final MechanismLigament2d intakeViz = intakeRoot
            .append(new MechanismLigament2d("intakeViz", 0.07, 0, 10, red));

    private final FlywheelSim intakeSim = new FlywheelSim(
                LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), 0.001, 5), DCMotor.getNEO(1));
    
    /*Constructor */
    public Simulation(AlgaeSubsystem algaeSubsystem, CoralSubsystem coralSubsystem, ElevatorSubsystem elevatorSubsystem){
        this.algaeSubsys = algaeSubsystem;
        this.coralSubsys = coralSubsystem;
        this.elevSubsys = elevatorSubsystem; 

        elevViz = elevRoot.append(new MechanismLigament2d("elevatorLigament", 0, 90, 70.0,
                new Color8Bit(Color.kBlanchedAlmond)));
        pivotViz = pivotRoot
            .append(new MechanismLigament2d("pivotViz", 0.56, algaeSubsys.startingAngle));

        SmartDashboard.putData("elevSimMech", elevMech);
        SmartDashboard.putData("coralIntake", coralIntakeMech);
        SmartDashboard.putData("Algae Subsys Viz", algaeMech);
    }


    @Override
    public void simulationPeriodic(){
        SmartDashboard.putNumber("Sim Algae Pivot Viz Angle", pivotViz.getAngle());
        SmartDashboard.putNumber("Sim Algae Intake Viz Angle", intakeViz.getAngle());
        SmartDashboard.putNumber("Sim left intake viz angle", leftIntakeViz.getAngle());
        SmartDashboard.putNumber("Sim right intake viz angle", rightIntakeViz.getAngle());

       
       
        elevViz.setLength(elevSubsys.simElevEncoder);


        intakeSim.setInput(algaeSubsys.intakeSpeed);
        pivotViz.setAngle(algaeSubsys.simPivotEncoder);

        intakeSim.update(0.005);
        intakeViz.setAngle(intakeViz.getAngle() + (algaeSubsys.intakeSpeed * 5));
        // woah so cool. uses trigonometry so the intake wheel can follow the pivot on the sim viz
        intakeRoot.setPosition((Math.cos(Units.degreesToRadians(pivotViz.getAngle())) * 0.56) + 0.75,
                (Math.sin(Units.degreesToRadians(pivotViz.getAngle())) * 0.56) + 0.75);
      
        var leftCurrentPos = leftIntakeViz.getAngle();
        var rightCurrentPos = rightIntakeViz.getAngle();

        intakeLeftSim.setInput(coralSubsys.motorSpeed);
        intakeLeftSim.update(0.001);

            intakeRightSim.setInput(coralSubsys.secondaryMotorSpeed);
            intakeRightSim.update(0.001);
        

        leftIntakeViz.setAngle(leftCurrentPos + (intakeLeftSim.getAngularVelocityRPM() * 0.07));
        rightIntakeViz.setAngle(rightCurrentPos - (intakeRightSim.getAngularVelocityRPM() * 0.07));

        if(coralSubsys.inSensor()){
            inSensorViz.setColor(green);
        } else{
            inSensorViz.setColor(red);
        }

        if(coralSubsys.outSensor()){
            outSensorViz.setColor(green);
        } else{
            outSensorViz.setColor(red);
        }

        algaePivotPublisher.set(new Pose3d(0, 0, 0, new Rotation3d(0, pivotViz.getAngle(), 0)));
        algaeIntakePublisher.set(new Pose3d(0, 0, 0, new Rotation3d(0, intakeViz.getAngle(),0)));
        coralLeftWheelsPublisher.set(new Pose3d(0, 0, 0, new Rotation3d(0, 0, leftIntakeViz.getAngle())));
        coralRightWheelsPublisher.set(new Pose3d(0, 0, 0, new Rotation3d(0, 0, rightIntakeViz.getAngle())));
        elevatorPublisher.set(new Pose3d(0, 0, elevViz.getLength(), new Rotation3d(0, 0, 0)));
    }
    
}
