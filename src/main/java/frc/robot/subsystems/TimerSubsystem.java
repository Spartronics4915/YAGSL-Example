package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Commands;

public class TimerSubsystem extends SubsystemBase {

    private double startTime;
    public TimerSubsystem() {


    }

    public Command startTimerCommand() {
        return Commands.runOnce(() -> {startTime = Timer.getFPGATimestamp();});    
    }

    public Command printElapsedTimeCommand() {

        return Commands.runOnce(()->{System.out.println("Elapse time: " + (Timer.getFPGATimestamp()-startTime));});
    }
    
}
