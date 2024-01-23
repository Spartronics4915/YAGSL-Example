package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import com.choreo.lib.Choreo;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.rollers.IntakeRollerSubsystem.IntakeState;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import edu.wpi.first.math.geometry.Translation2d;

public class ChoreoPickUpNoteCommand {

    public static Command createChoreoPickUpNoteCommand(SwerveSubsystem drive, String trajName, Translation2d noteLocation,
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
        Command triggerCommand = new TriggerIntakeDeployCommand(drive, intake, noteLocation, 1, true);

        return Commands.parallel(swerveCommand, triggerCommand);

    }

    public static class TriggerIntakeDeployCommand extends Command {
        SwerveSubsystem mDrive;
        Translation2d mTriggerLocation;
        double mTriggerDistance;
        IntakeSubsystem mIntake;
        boolean mIsFinished;
        boolean mActivateIntake;

        public TriggerIntakeDeployCommand(SwerveSubsystem drive, IntakeSubsystem intake, Translation2d triggerLocation, double triggerDistance,
        boolean activateIntake) {
            mDrive = drive;
            mTriggerLocation = triggerLocation;
            mTriggerDistance = triggerDistance;
            mIsFinished = false;
            mIntake = intake;
            mActivateIntake = activateIntake;
        }

        @Override
        public void execute() {
            Translation2d currLoc = mDrive.getPose().getTranslation();
            double distToTrigger = mTriggerLocation.getDistance(currLoc);
            if (distToTrigger < mTriggerDistance) {

                mIntake.deployShooterGroundPickup();
                mIsFinished = true;
                if(mActivateIntake) {
                    mIntake.rollerSubsystem.setIntakeRollerState(IntakeState.INTAKE);
                }
            }

        }

        @Override
        public boolean isFinished() {
            return mIsFinished;
        }
    }

}
