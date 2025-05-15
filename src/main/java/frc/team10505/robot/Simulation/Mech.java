package frc.team10505.robot.Simulation;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class Mech {
    private Mechanism2d mech;
    public Viz viz;
    public Viz viz2;
    public Viz viz3;
    public Viz viz4;
  
    private String name;
  
    public Mech(String subsystemName, double mechWidth, double mechHeight) {
        mech = new Mechanism2d(mechWidth, mechHeight);
        this.name = subsystemName;
    }

    public Mechanism2d getMech(){
        return mech;
    }

    public void addViz(String vizName, double rootX, double rootY, double ligLength, double ligAngle,
            double ligLineWidth, Color8Bit ligColor) {
         viz = new Viz(vizName, rootX, rootY, ligLength, ligAngle, ligLineWidth, ligColor);
    }

    public void addViz2(String vizName, double rootX, double rootY, double ligLength, double ligAngle,
            double ligLineWidth, Color8Bit ligColor) {
        viz2 = new Viz(vizName, rootX, rootY, ligLength, ligAngle, ligLineWidth, ligColor);
    }

    public void addViz3(String vizName, double rootX, double rootY, double ligLength, double ligAngle,
            double ligLineWidth, Color8Bit ligColor) {
        viz3 = new Viz(vizName, rootX, rootY, ligLength, ligAngle, ligLineWidth, ligColor);
    }

    public void addViz4(String vizName, double rootX, double rootY, double ligLength, double ligAngle,
            double ligLineWidth, Color8Bit ligColor) {
        viz4 = new Viz(vizName, rootX, rootY, ligLength, ligAngle, ligLineWidth, ligColor);
    }

    public class Viz {
        private MechanismRoot2d root;
        private MechanismLigament2d ligament;

        private StructPublisher<Pose3d> publisher;

        private Pose3d currentPublisherPose = new Pose3d(0,0,0, new Rotation3d(0,0,0));

        public Viz(String vizName, double rootX, double rootY, double ligLength, double ligAngle, double ligLineWidth,
                Color8Bit ligColor) {
            root = mech.getRoot(name + " " + vizName + " Root", rootX, rootY);
            ligament = root.append(new MechanismLigament2d(name + " " + vizName + " Viz", ligLength, ligAngle, ligLineWidth,
                    ligColor));
            publisher = NetworkTableInstance.getDefault().getStructTopic(name + " " + vizName + " Publisher", Pose3d.struct)
                    .publish();
        }

        public MechanismLigament2d getLigament() {
            return ligament;
        }

        public double getAngle(){
            return ligament.getAngle();
        }

        public void setColor(Color8Bit color) {
            ligament.setColor(color);
        }

        public void setLength(double length) {
            ligament.setLength(length);
            currentPublisherPose.transformBy(new Transform3d(0,0,length, new Rotation3d(0,0,0)));
        }

        public void setAngle(double angle, Rotation3d rotation) {
            ligament.setAngle(angle);
            currentPublisherPose.transformBy(new Transform3d(0,0,0, rotation));
        }

        public void setRoot(double x, double y) {
            root.setPosition(x, y);
            currentPublisherPose.transformBy(new Transform3d(x, 0, y, new Rotation3d()));
        }

        /**Should be called periodically to keep publisher pose updated */
        public void updatePublisher(){
            publisher.set(currentPublisherPose);
        }

        public void setPublisherPose(Pose3d pose){
            publisher.set(pose);
        }
    }
}
