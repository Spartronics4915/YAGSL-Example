package frc.robot.commands.swervedrive.auto;

import com.choreo.lib.ChoreoTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class PrintDistFromTrajBegin {
    
    public static Translation2d getDiffFromTrajStart(SwerveSubsystem swerve, ChoreoTrajectory traj) {
        Pose2d currPose = swerve.getPose();
        Translation2d currPos = currPose.getTranslation();
        
        Translation2d trajInitialPos= traj.getInitialPose().getTranslation();

        return trajInitialPos.minus(currPos);

    }

    public static Command buildPrintDistFromTrajStart(SwerveSubsystem swerve, ChoreoTrajectory traj) {
        
        Runnable cmdFun = () -> {
            System.out.println("Dist from next start:" + getDiffFromTrajStart(swerve, traj).getNorm());
        };
        
        return Commands.runOnce(cmdFun);
    }
}
