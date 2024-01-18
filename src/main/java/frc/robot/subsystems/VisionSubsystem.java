package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class VisionSubsystem extends SubsystemBase {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tv = table.getEntry("tv");
    NetworkTableEntry tx = table.getEntry("tx");
    // NetworkTableEntry ty = table.getEntry("ty");
    // NetworkTableEntry ta = table.getEntry("ta");
    NetworkTableEntry botpose = table.getEntry("botpose");

    double[] botposeArray = botpose.getDoubleArray(new double[7]);
    double x = botposeArray[0];
    double y = botposeArray[1];
    double z = botposeArray[2];

    // //read values periodically
    // double x = tx.getDouble(0.0);
    // double y = ty.getDouble(0.0);
    // double area = ta.getDouble(0.0);

    // //post to smart dashboard periodically
    // SmartDashboard.putNumber("LimelightX", x);
    // SmartDashboard.putNumber("LimelightY", y);
    // SmartDashboard.putNumber("LimelightArea", area);
    public boolean isLookingAtAprilTag() {
        // System.out.println("isLookingAtAprilTag method triggered");
        // System.out.println("tv: " + tv.getDouble(0));
        // System.out.println("tx: " + tx.getDouble(0));
        // System.out.println("ty: " + ty.getDouble(0));
        // System.out.println("ta: " + ta.getDouble(0));
        // System.out.println("tv 2: " + ta.getDouble(2));
        return tv.getDouble(0) == 1;
    }

    public double getTx() {
        return tx.getDouble(0);
    }
}