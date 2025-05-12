package frc.team10505.robot.libTypeStuff;

import java.lang.reflect.Type;

import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class Mech {
    private StructPublisher<Pose3d> publisher;
    private LoggedMechanism2d mech;
    private String name;
    public Viz viz1;
    public Viz viz2;
    public Viz viz3;
    public Viz viz4;
    private Type bruh;


    public static enum Type {
        viz1,
        viz2,
        viz3,
        viz4,
    }

    // private LoggedMechanismRoot2d root;
    // private LoggedMechanismLigament2d ligament;

    public Mech(String name, double mechWidth, double mechHeight) {
        mech = new LoggedMechanism2d(mechWidth, mechHeight);
        this.name = name;
    }

    public void addViz(String vizName, double rootX, double rootY, double ligLength, double ligAngle,
            double ligLineWidth, Color8Bit ligColor) {
        Viz bruh = new Viz(vizName, rootX, rootY, ligLength, ligAngle, ligLineWidth, ligColor);
    }

    // public void addViz2(String vizName, double rootX, double rootY, double ligLength, double ligAngle,
    //         double ligLineWidth, Color8Bit ligColor) {
    //     viz2 = new Viz(vizName, rootX, rootY, ligLength, ligAngle, ligLineWidth, ligColor);
    // }

    // public void addViz3(String vizName, double rootX, double rootY, double ligLength, double ligAngle,
    //         double ligLineWidth, Color8Bit ligColor) {
    //     viz3 = new Viz(vizName, rootX, rootY, ligLength, ligAngle, ligLineWidth, ligColor);
    // }

    // public void addViz4(String vizName, double rootX, double rootY, double ligLength, double ligAngle,
    //         double ligLineWidth, Color8Bit ligColor) {
    //     viz4 = new Viz(vizName, rootX, rootY, ligLength, ligAngle, ligLineWidth, ligColor);
    // }

    public class Viz {
        private LoggedMechanismRoot2d root;
        private LoggedMechanismLigament2d ligament;

        public Viz(String vizName, double rootX, double rootY, double ligLength, double ligAngle, double ligLineWidth,
                Color8Bit ligColor) {
            mech.getRoot(name + vizName + " Root", rootX, rootY);
            root.append(new LoggedMechanismLigament2d(name + vizName + " Viz", ligLength, ligAngle, ligLineWidth,
                    ligColor));
            publisher = NetworkTableInstance.getDefault().getStructTopic(vizName + " Publisher", Pose3d.struct)
                    .publish();
        }

        public LoggedMechanismLigament2d getLigament() {
            return ligament;
        }

        public void setColor(Color8Bit color) {
            ligament.setColor(color);
        }

        public void setLength(double length) {
            ligament.setLength(length);
        }

        public void setAngle(double angle) {
            ligament.setAngle(angle);
        }

        public void setLineWeight(double lineWeight) {
            ligament.setLineWeight(lineWeight);
        }

        public void setRoot(double x, double y) {
            root.setPosition(x, y);
        }
    }

}
