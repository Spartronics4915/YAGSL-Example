package frc.robot.subsystems.wrists;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeWristSubsystem extends SubsystemBase {

    final double maxAngle = 115; // Degrees
    final double minAngle = 0;
    final TrapezoidProfile.Constraints motionConstraints = new TrapezoidProfile.Constraints(maxAngle/3, maxAngle/6);
    final TrapezoidProfile motionProfile = new TrapezoidProfile(motionConstraints);

    double currUserSetPoint = 0;
    double currProfileSetPoint = 0;
    double currPredictedVelocity = 0;
    boolean m_isActive;

    public IntakeWristSubsystem() {
        m_isActive = false;

        initializePositions();
    }

    private void initializePositions() {

        if (RobotBase.isSimulation()) {
            currUserSetPoint = 0;
            currProfileSetPoint = 0;
            currPredictedVelocity = 0;
        }
    }

    public double getUserSetPoint() {
        return currUserSetPoint;
    }

    public void setUserSetPoint(double userSetPoint) {

        if (userSetPoint > maxAngle) {

            currUserSetPoint = maxAngle;
        } else if (userSetPoint < minAngle) {
            userSetPoint = minAngle;
        } else {
            currUserSetPoint = userSetPoint;
        }

    }

    /*
     * Returns the setpoint used by the motion profile. Only use this if you know
     * that you want it.
     * 
     */
    public double getProfileSetPoint() {
        return currProfileSetPoint;
    }

    public boolean isActive() {
        return m_isActive;
    }

    public void activate() {
        m_isActive = true;
    }

    public void deactivate() {
        m_isActive = false;
    }
    
    public void periodic() {

        final double dT = 1. / 20;

        if (m_isActive) {
            //System.out.println(currProfileSetPoint + " " + currUserSetPoint);
            TrapezoidProfile.State currState = new TrapezoidProfile.State(currProfileSetPoint, currPredictedVelocity);
            TrapezoidProfile.State goalState = new TrapezoidProfile.State(currUserSetPoint, 0);
            TrapezoidProfile.State nextState = motionProfile.calculate(dT, currState, goalState);
            currPredictedVelocity = nextState.velocity;
            currProfileSetPoint = nextState.position;
        }
    }

    // A test command to verify the system moves

    private void pingPongFunction() {
        final double span = maxAngle - minAngle;
        if (m_isActive) {
            if ((getProfileSetPoint() - minAngle) < 0.05 * span) {

                setUserSetPoint(maxAngle);
            } else if ((maxAngle - getProfileSetPoint()) < 0.05 * span) {
                setUserSetPoint(minAngle);
            }
        }
    }

    public Command pingPongCommand() {

        return this.run(() -> pingPongFunction());
    }

}
