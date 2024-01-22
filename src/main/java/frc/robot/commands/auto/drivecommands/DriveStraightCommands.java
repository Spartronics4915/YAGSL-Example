package frc.robot.commands.auto.drivecommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
public class DriveStraightCommands{

    public static class DriveStraightFixedDistance extends Command {

        double mDistance;
        Rotation2d mHeading;
        SwerveSubsystem mSwerveSubsystem;
        double headingX, headingY;
        Pose2d initialPt;
        TrapezoidProfile.Constraints mConstraints;
        double mDistTraveled;
        TrapezoidProfile mProfile;
        TrapezoidProfile.State mGoalState;
        /*Drives straight in a direction, the heading is CCW+ with 0 driving straight forward */
        public DriveStraightFixedDistance(SwerveSubsystem swerveSubsystem, Rotation2d heading, double distance,
        TrapezoidProfile.Constraints constraints) {
            mDistance = distance;
            mHeading = heading;
            mSwerveSubsystem = swerveSubsystem;
            mConstraints = constraints;
            addRequirements(mSwerveSubsystem);
            mProfile = new TrapezoidProfile(mConstraints);
            headingX = heading.getCos();
            headingY = heading.getSin();
            mGoalState = new TrapezoidProfile.State(distance, 0);
        }

        public void initialize() {
            initialPt = mSwerveSubsystem.getPose();
        }

        public void execute() {

            final double dT = 1./50;
            Pose2d currPt = mSwerveSubsystem.getPose();
            mDistTraveled = currPt.getTranslation().minus(initialPt.getTranslation()).getNorm();
            ChassisSpeeds currRobotVelocity = mSwerveSubsystem.getRobotVelocity();
            Translation2d robotVelocity = new Translation2d(currRobotVelocity.vxMetersPerSecond, currRobotVelocity.vyMetersPerSecond);
            TrapezoidProfile.State currState = new TrapezoidProfile.State(mDistTraveled, robotVelocity.getNorm());
            TrapezoidProfile.State newState = mProfile.calculate(dT, currState, mGoalState);

            mSwerveSubsystem.drive();
        }
        
    }

}
