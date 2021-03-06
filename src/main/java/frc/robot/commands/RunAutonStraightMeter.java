// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

import java.awt.Color;
import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrain;
//mport frckit.tools.pathview.TrajectoryMarker;
import frc.robot.commands.NewRunMotionProfile.CirclePath;
import frc.robot.Constants;
//import frc.robot.Constants.RobotType;
import frc.robot.subsystems.RobotOdometry;
import frckit.tools.pathview.TrajectoryMarker;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RunAutonStraightMeter extends SequentialCommandGroup {

  NewRunMotionProfile mp;
  private static final double markerDiameter = 4;
  private static final Color markerColor = Color.BLACK;

  /** Creates a new RunAutoNavSlalom. */
  public RunAutonStraightMeter(RobotOdometry odometry, DriveTrain driveTrain) {
      System.out.println("Attempting to start auton"); // auton actually starts so theres a differing issue
    mp = new NewRunMotionProfile(driveTrain, odometry, 0.0,
        List.of(new Pose2d(0, 0, new Rotation2d()), new Pose2d(40.0, 0, Rotation2d.fromDegrees(0))),
        Double.MAX_VALUE, false, false);

        //System.out.println(odometry.getCurrentPose());
    // Add your addCommands(new FooCommand(), new BarCommand());
    addCommands(new InstantCommand(() -> odometry.setPosition(new Pose2d(30, 30, new Rotation2d()))), mp,
        new InstantCommand(() -> driveTrain.stop()));
  }

  
  
  public static void main(String[] args) {
    //Constants.setRobot(RobotType.ROBOT_2020);
    RunAutonStraightMeter cmd = new RunAutonStraightMeter(null, null);
    cmd.mp.visualize(80,
        List.of(new TrajectoryMarker(new Translation2d(30, 120), markerDiameter, markerColor),
            new TrajectoryMarker(new Translation2d(60, 120), markerDiameter, markerColor),
            new TrajectoryMarker(new Translation2d(30, 60), markerDiameter, markerColor),
            new TrajectoryMarker(new Translation2d(60, 60), markerDiameter, markerColor),
            new TrajectoryMarker(new Translation2d(120, 60), markerDiameter, markerColor),
            new TrajectoryMarker(new Translation2d(150, 60), markerDiameter, markerColor),
            new TrajectoryMarker(new Translation2d(180, 60), markerDiameter, markerColor),
            new TrajectoryMarker(new Translation2d(210, 60), markerDiameter, markerColor),
            new TrajectoryMarker(new Translation2d(240, 60), markerDiameter, markerColor),
            new TrajectoryMarker(new Translation2d(300, 60), markerDiameter, markerColor)));
  }
  
}
