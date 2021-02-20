package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.Constants;

public class SwerveDriveCommand extends CommandBase {

  private final SwerveDrivetrain drivetrain;
  private final XboxController controller;

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter rotLimiter = new SlewRateLimiter(3);

  public SwerveDriveCommand(SwerveDrivetrain drivetrain, XboxController controller) {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);

    this.controller = controller;
  }

  @Override
  public void execute() {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    //Driver controller left joystick y
    final var xSpeed =
      -xspeedLimiter.calculate(Robot.robotContainer.getLeftXboxJoystickValue())
        * Constants.kMaxSpeed;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    //Driver controller left joystick x
    final var ySpeed =
      -yspeedLimiter.calculate(Robot.robotContainer.getLeftXboxJoystickValueX())
        * Constants.kMaxSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    //right x
    final var rot =
      -rotLimiter.calculate(Robot.robotContainer.getRightXboxJoystickValueX())
        * Constants.kMaxAngularSpeed;
    //right 
    boolean fieldRelative = true; // this value is supposed to be a bool value of the right bumper

    drivetrain.drive(xSpeed, ySpeed, rot, fieldRelative);
  }

}
