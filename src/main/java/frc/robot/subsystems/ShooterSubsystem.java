package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.wrists.ShooterWristSubsystem;

public class ShooterSubsystem extends SubsystemBase {

    public ShooterWristSubsystem shooterWrist;
    public ShooterSubsystem () {

        shooterWrist = new ShooterWristSubsystem();
    }

    public void activateSystems() {
        shooterWrist.activate();
    }
    
}
