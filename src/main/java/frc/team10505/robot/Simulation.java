package frc.team10505.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team10505.robot.libTypeStuff.Mech;
import frc.team10505.robot.subsystems.AlgaeSubsystem;
import frc.team10505.robot.subsystems.CoralSubsystem;
import frc.team10505.robot.subsystems.ElevatorSubsystem;

public class Simulation extends SubsystemBase {
    private AlgaeSubsystem algaeSubsys;
    private CoralSubsystem coralSubsys;
    private ElevatorSubsystem elevSubsys;

    // Colors
    private final Color8Bit red = new Color8Bit(Color.kFirstRed);
    private final Color8Bit green = new Color8Bit(Color.kGreen);
    private final Color8Bit blue = new Color8Bit(Color.kSkyBlue);
    private final Color8Bit orange = new Color8Bit(Color.kPapayaWhip);

    // mechs
    private final Mech elevMech = new Mech("Elevator", 1.5, 1.5);

    private final Mech algaeMech = new Mech("Algae", 1.5, 1.5);

    private final Mech coralMech = new Mech("Coral", 1.5, 1.5);

    // 2d flywheel sims (Can be outside of the subsystem, as is isnt a feedback
    // loop)
    private final FlywheelSim intakeLeftSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), 0.003, 5), DCMotor.getNEO(1));

    private final FlywheelSim intakeRightSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), 0.003, 5), DCMotor.getNEO(1));

    private final FlywheelSim intakeSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), 0.001, 5), DCMotor.getNEO(1));

    /* Constructor */
    public Simulation(AlgaeSubsystem algaeSubsystem, CoralSubsystem coralSubsystem,
            ElevatorSubsystem elevatorSubsystem) {
        this.algaeSubsys = algaeSubsystem;
        this.coralSubsys = coralSubsystem;
        this.elevSubsys = elevatorSubsystem;

        elevMech.addViz("Carriage", 0.75, 0.1, 0, 90, 70.0, orange);

        algaeMech.addViz("Pivot", 0.75, 0.75, 0.56, algaeSubsys.startingAngle, 10, orange);

        algaeMech.addViz2("Wheel", 1.5, 1.5, 0.07, 0, 10, blue);

        coralMech.addViz("Left Intake", 0.6, 0.6, 0.4, 0, 10, orange);

        coralMech.addViz2("Right Intake", 2.4, 0.6, 0.4, 180, 10, orange);

        coralMech.addViz3("In Sensor", 1.5, 1, 0.15, 90, 40, red);

        coralMech.addViz4("Out Sensor", 1.5, 0.2, 0.15, -90, 40, red);

        SmartDashboard.putData("ELEV_MECH", elevMech.getMech());
        SmartDashboard.putData("ALGAE_MECH", algaeMech.getMech());
        SmartDashboard.putData("CORAL_MECH", coralMech.getMech());

    }

    public double getMechanismCurrentDrawAms() {
        return algaeSubsys.pivotSim.getCurrentDrawAmps()
                + intakeSim.getCurrentDrawAmps()
                + intakeLeftSim.getCurrentDrawAmps()
                + intakeRightSim.getCurrentDrawAmps()
                + elevSubsys.elevSim.getCurrentDrawAmps();
    }

    @Override
    public void simulationPeriodic() {
        //flywheel sim updates
        intakeSim.setInput(algaeSubsys.intakeSpeed);
        intakeSim.update(0.001);

        intakeLeftSim.setInput(coralSubsys.motorSpeed);
        intakeLeftSim.update(0.001);

        intakeRightSim.setInput(coralSubsys.secondaryMotorSpeed);
        intakeRightSim.update(0.001);

        //viz updates
        elevMech.viz.setLength(elevSubsys.simElevEncoder);

        algaeMech.viz.setAngle(algaeSubsys.simPivotEncoder, new Rotation3d(0, algaeSubsys.simPivotEncoder, 0));

        algaeMech.viz2.setAngle(algaeMech.viz2.getAngle() + algaeSubsys.intakeSpeed * 12, new Rotation3d(0, algaeMech.viz2.getAngle(), 0));

        algaeMech.viz2.setRoot((Math.cos(Units.degreesToRadians(algaeMech.viz.getAngle())) * 0.56) + 0.75,
        (Math.sin(Units.degreesToRadians(algaeMech.viz.getAngle())) * 0.56) + 0.75);

        coralMech.viz.setAngle(coralMech.viz.getAngle() + intakeLeftSim.getAngularVelocityRPM() * 0.2, new Rotation3d(0, 0, coralMech.viz.getAngle()));

        coralMech.viz2.setAngle(coralMech.viz2.getAngle() - intakeRightSim.getAngularVelocityRPM() * 0.2, new Rotation3d(0, 0, coralMech.viz2.getAngle()));
 
        if (coralSubsys.inSensor()) {
            coralMech.viz3.setColor(green);
        } else {
            coralMech.viz3.setColor(red);
        }

        if (coralSubsys.outSensor()) {
            coralMech.viz4.setColor(green);
        } else {
            coralMech.viz4.setColor(red);
        }

        elevMech.viz.updatePublisher();
        algaeMech.viz.updatePublisher();
        algaeMech.viz2.updatePublisher();
        coralMech.viz.updatePublisher();
        coralMech.viz2.updatePublisher();
    }

}
