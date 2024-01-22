package frc.robot.subsystems;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {

    final TrapezoidProfile.Constraints motionConstraints = new TrapezoidProfile.Constraints(0.25, 0.125);
    final TrapezoidProfile motionProfile = new TrapezoidProfile(motionConstraints);
    final double maxHeight = 1.5; // Meters

    double currUserSetPoint = 0;
    double currProfileSetPoint = 0;
    boolean m_isActive;

    public ElevatorSubsystem() {
        m_isActive = false;

        initializePositions();
    }

    private void initializePositions() {

        if (RobotBase.isSimulation()) {
            currUserSetPoint = 0;
            currProfileSetPoint = 0;
        }
    }

    public double getUserSetPoint() {
        return currUserSetPoint;
    }

    /*
     * Returns the setpoint used by the motion profile. Only use this if you know
     * that you want it.
     * 
     */
    public double getProfileSetPoint() {
        return currProfileSetPoint;
    }

    public void periodic() {

        final double dT = 1. / 20;
        
    }

}
