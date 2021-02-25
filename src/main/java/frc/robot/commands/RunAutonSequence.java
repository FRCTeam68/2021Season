/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.paths.SixBallp1;
import frc.paths.SixBallp2;
import frc.robot.commands.PathFollower;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class RunAutonSequence extends SequentialCommandGroup {
  /**
   * Creates a new PP.
   */
  public RunAutonSequence() {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(    
      new ParallelCommandGroup(
        new ChangeIntakePos(),
        new ShootMedium()
        ),
        new PathFollower(new SixBallp1()),
        new AutonLock(),
        new SetAgitator(),
        new WaitCommand(1.5),
        new ParallelCommandGroup(new Zero(), new SetIntakeWithDouble(-1), new AutonLockZero()),
        new ParallelCommandGroup(new ShootMedium()),
        new PathFollower(new SixBallp2()),
        new ParallelCommandGroup(new SetIntakeWithDouble(0)),
        new PathFollower(new SixBallp2()).reverse(),
        new AutonLock(),
        new SetAgitator());

    
  }
}
