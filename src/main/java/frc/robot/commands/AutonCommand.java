package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.*;
import frc.robot.Robot;

public class AutonCommand extends SequentialCommandGroup{
    
    public AutonCommand(){
        addCommands(new ChangeIntakePos(),
        new AutonLock(),
        new ShootMedium(),
        new WaitCommand(2),
        new SpinFeeder(.75),
        new WaitCommand(4),
        new SpinFeeder(0),
        new Zero(),
        new AutonLockZero(),
        new WaitCommand(1),
        new RunAutonStraightMeter(Robot.robotOdemetry, Robot.driveTrain)); 
    }

    
}
