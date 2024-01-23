package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.wpilibj2.command.Command;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public class ChoreoPickUpNoteCommand {

    public Command createChoreoPickUpNoteCommand(SwerveSubsystem drive, String trajName, Pose2d noteLocation,
            IntakeSubsystem intake) {
        var traj = Choreo.getTrajectory(trajName);
        var thetaController = new PIDController(0, 0, 0);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        Command swerveCommand = Choreo.choreoSwerveCommand(
                traj, // Choreo trajectory from above
                drive::getPose, // A function that returns the current field-relative pose of the robot: your
                // wheel or vision odometry
                new PIDController(1, 0.0, 0.0), // PIDController for field-relative X
                // translation (input: X error in meters,
                // output: m/s).
                new PIDController(1, 0.0, 0.0), // PIDController for field-relative Y
                // translation (input: Y error in meters,
                // output: m/s).
                thetaController, // PID constants to correct for rotation
                // error
                (ChassisSpeeds speeds) -> {

                    drive.drive(speeds);
                },
                () -> false, // Whether or not to mirror the path based on alliance (this assumes the path is
                             // created for the blue alliance)
                drive // The subsystem(s) to require, typically your drive subsystem only
        );

        return null;

    }

    public class TriggerIntakeCommand extends Command {
        SwerveSubsystem mDrive;
        Translation2d mTriggerLocation;
        double mTriggerDistance;
        IntakeSubsystem mIntake;

        public TriggerIntakeCommand(SwerveSubsystem drive, IntakeSubsystem intake, Translation2d triggerLocation, double triggerDistance) {
            mDrive = drive;
            mTriggerLocation = triggerLocation;
            mTriggerDistance = triggerDistance;
        }

        @Override
        public void execute() {
            Translation2d currLoc = mDrive.getPose().getTranslation();
            double distToTrigger = mTriggerLocation.getDistance(currLoc);
            if (distToTrigger < mTriggerDistance) {

                
            }

        }
    }

}
