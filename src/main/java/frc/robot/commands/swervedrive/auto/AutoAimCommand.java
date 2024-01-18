package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoAimConstants;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class AutoAimCommand extends Command {
    private final SwerveSubsystem mSwerve;
    private final VisionSubsystem mVision;

    public AutoAimCommand(SwerveSubsystem mSwerve, VisionSubsystem mVision) {
        super();
        this.mSwerve = mSwerve;
        this.mVision = mVision;
    }

    private double getTx(){
        if (mVision.isLookingAtAprilTag()) {
            return mVision.getTx();
        } else {
            return 0.0;
        }
    }

    @Override
    public void execute() {
        mSwerve.drive(new Translation2d(0, 0), getTx() * AutoAimConstants.kP, false);
    }
    @Override
    public boolean isFinished() {
        return (getTx() < 0.05);
    }

    
}
