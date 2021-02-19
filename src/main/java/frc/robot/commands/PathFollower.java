/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.team2363.commands.HelixFollower;
import frc.team2363.controller.PIDController;

import com.team319.trajectory.Path;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

import frc.robot.Robot;

public class PathFollower extends HelixFollower  {


    // These are the 2 PID controllers that will handle error for your total travel distance and heading

     PIDController headingController = new PIDController(Constants.AUTON_ANGLE_KP, Constants.AUTON_ANGLE_KI, Constants.AUTON_ANGLE_KD, 0.001);
     PIDController distanceController = new PIDController(Constants.AUTON_DISTANCE_KP, Constants.AUTON_DISTANCE_KI, Constants.AUTON_DISTANCE_KD, 0.001);

    public PathFollower(Path path) {
        super(path);
        System.out.println("STARTING TRAJECTORY");
        // Make sure to require your subsystem so you don't have conflicting commands
        addRequirements(Robot.driveTrain);
    }

	@Override
    public void resetDistance() {
        // We need to reset the encoders back to 0 at the start of the path
        Robot.driveTrain.ResetEncoders();
       // Robot.driveTrain
    }

    @Override
    public PIDController getHeadingController() {
        // Here we return the PID controller that we're using to correct the heading error through the path
        return headingController;
    }

    @Override
    public PIDController getDistanceController() {
        // Here we return the PID controller that we're using to correct the distance error through the path
        return distanceController;
    }

    @Override
    public double getCurrentDistance() {
        // Here we need to return the overall robot distance traveled in FEET in this example we are averaging 
        // the two sides of the DriveTrain to give is the robot's distance travelled
        //SmartDashboard.putNumber("DISTANCE TRAVELED", (Robot.driveTrain.getLeftEnc()/Constants.ENCODER_TICK_LEFT_REVOLUTION + Robot.driveTrain.getRightEnc()/Constants.ENCODER_TICK_RIGHT_REVOLUTION) / 2.0);
        return (Robot.driveTrain.getLeftEnc()/(Constants.ENCODER_TICK_LEFT_REVOLUTION/(3.14159*Constants.WHEEL_DIAMETER/12)) + Robot.driveTrain.getRightEnc()/Constants.ENCODER_TICK_RIGHT_REVOLUTION/(3.14159*Constants.WHEEL_DIAMETER/12)) / 2.0;
    }

    @Override
    public double getCurrentHeading() {
        // Here we need to return the current heading of the robot in RADIANS (positive counter-clockwise).
        double heading = (Robot.driveTrain.getYAW()-180)*(3.141592654/180);
        return heading;
    }

    @Override
    public void useOutputs(double left, double right) {
        // Here we will use the provided parameters in FPS and send them off to our DriveTrain. In this example 
        // the max velocity of our DriveTrain is 12 FPS. We are dividing the two provided parameters by the max 
        // veocity to convert them into a percentage and sending them off to our DriveTrain.
        SmartDashboard.putNumber("leftSpeed", left);
        SmartDashboard.putNumber("right speed", right);
        Robot.driveTrain.setSpeedFalcon(left/Constants.MAX_SPEED, right/Constants.MAX_SPEED);
        if(isFinished()){
        Robot.driveTrain.setSpeedFalcon(0, 0);
        }
    }

}