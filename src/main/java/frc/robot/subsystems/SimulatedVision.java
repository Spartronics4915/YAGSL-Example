package frc.robot.subsystems;

import edu.wpi.first.wpilibj.RobotBase;
import swervelib.SwerveDrive;

public class SimulatedVision {
    
    // The drive is needed for odometry
    SwerveDrive swerveDrive;
    SimulatedVision(SwerveDrive drive) {

        swerveDrive = drive;
    }

    double getDegreesFromSpeaker(){return 0;}

}
