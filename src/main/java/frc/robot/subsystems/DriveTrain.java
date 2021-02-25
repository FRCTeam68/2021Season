/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.DriveWithJoysticks;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
public class DriveTrain extends SubsystemBase {
  /**
   * Creates a new DriveTrain.
   */ 
  private WPI_TalonFX fr; //front right
  private WPI_TalonFX br; //back right
  private WPI_TalonFX bl; //back left
  private WPI_TalonFX fl; //front left\
  
  /*
  private CANCoder leftDriveEnc;
  private CANCoder RightDriveEnc;
  */

  private final DifferentialDriveOdometry m_odometry;

  
  NetworkTableEntry m_xEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("X");
  NetworkTableEntry m_yEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("Y");
  


// The robot's drive

// The left-side drive encoder


// The right-side drive encoder

// The gyro sensor
private AHRS m_gyro = new AHRS();

  public static DriveTrain driveTrain;


	public static DriveTrain getDriveTrain() {
		if (driveTrain == null) {
			driveTrain = new DriveTrain();
		}
		return driveTrain;
	}
  public DriveTrain() {
    // NAVX CONFIG
    m_gyro = new AHRS(SPI.Port.kMXP);
    //m_gyro.reset();
    
		
    // DriveTrain Motors Config
    fr = new WPI_TalonFX(Constants.TALONFX_FR);
    br = new WPI_TalonFX(Constants.TALONFX_BR);
    bl = new WPI_TalonFX(Constants.TALONFX_BL);
    fl = new WPI_TalonFX(Constants.TALONFX_FL);
    /*
    CAN ENCODER GETS DEGREES NOT ENCODER TICKS
    leftDriveEnc = new CANCoder(Constants.CANENCODER_LEFT_DRIVE);
    RightDriveEnc = new CANCoder(Constants.CANENCODER_RIGHT_DRIVE);
    */
    br.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor,0,0);
    br.selectProfileSlot(Constants.DRIVETRAIN_RIGHT_PID_SLOT, 0);
    br.setSensorPhase(true);
    br.setInverted(true);
    fr.setInverted(true);

    fl.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor,0,0);
    fl.selectProfileSlot(Constants.DRIVETRAIN_LEFT_PID_SLOT, 0);
    fl.setSensorPhase(false);
    //fl.setInverted(true);
    

    bl.set(ControlMode.Follower, fl.getDeviceID());
    fr.set(ControlMode.Follower, br.getDeviceID());


    fr.setNeutralMode(NeutralMode.Brake);
    br.setNeutralMode(NeutralMode.Brake);
    fl.setNeutralMode(NeutralMode.Brake);
    bl.setNeutralMode(NeutralMode.Brake);


    fr.configPeakOutputForward(1);
    br.configPeakOutputForward(1);
    fl.configPeakOutputForward(1);
    bl.configPeakOutputForward(1);
    fr.configPeakOutputReverse(-1);
    br.configPeakOutputReverse(-1);
    bl.configPeakOutputReverse(-1);
    fl.configPeakOutputReverse(-1);

    //m_odometry.resetPosition(, m_gyro.getYaw());
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
    

  }
  @Override
  public void periodic() {
    CommandScheduler.getInstance().setDefaultCommand(Robot.driveTrain, new DriveWithJoysticks());
    m_odometry.update(Rotation2d.fromDegrees(getYAW()), Constants.ENCODER_TICK_LEFT_REVOLUTION/(3.14159*Constants.WHEEL_DIAMETER/12), Robot.driveTrain.getRightEnc()/Constants.ENCODER_TICK_RIGHT_REVOLUTION/(3.14159*Constants.WHEEL_DIAMETER/12));

    var translation = m_odometry.getPoseMeters().getTranslation();
    m_xEntry.setNumber(translation.getX());
    m_yEntry.setNumber(translation.getY());

    
    //SmartDashboard.putNumber("m_xEntry", m_xEntry);
  }

  public void setSpeedFalcon(double left, double right){
    
    fl.set(ControlMode.PercentOutput,left);
    br.set(ControlMode.PercentOutput,right);
    
  }
  public void setSpeedAuto(double left, double right){
   /*
    fl.set(ControlMode.Velocity,left*5000);
    br.set(ControlMode.Velocity,right*5000);
    
    /*
    fl.set(ControlMode.Velocity,left*200);
    br.set(ControlMode.Velocity,right*200);
    */
  }


  /**
   * Zeroes the heading of the robot.
   */
  public void ResetEncoders(){
    fl.setSelectedSensorPosition(0,0,0);
    br.setSelectedSensorPosition(0,0,0);
    
        /*m_gyro.reset(); for auton experimental we may have to reset thee navx to re run a new path */
  }

  public void resetYaw(){
    //m_gyro.reset();
    m_gyro.zeroYaw();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from and degree
   */
  public double getHeading() {
    return m_gyro.getAngle();
  }

  public double getYAW() {
    return m_gyro.getYaw();
  }
  public boolean isNavXReady(){
    return m_gyro.isCalibrating();
  }

  
  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getLeftEnc(){
    return fl.getSelectedSensorPosition(0);
  }
  public double getRightEnc(){
    return br.getSelectedSensorPosition(0);

  }
  public double leftVisionAdjusted(){
    double leftSpeed;
    double joystickSpeed;
    double visionCorrect;
    joystickSpeed = Robot.m_robotContainer.getLeftXboxJoystickValue();
    visionCorrect = Robot.vision.steeringAdjust();
    leftSpeed = joystickSpeed += visionCorrect;
    return leftSpeed;
  }
  public double rightVisionAdjusted(){
    double rightSpeed;
    double joystickSpeed;
    double visionCorrect;
    joystickSpeed = Robot.m_robotContainer.getRightXboxJoystickValue();
    visionCorrect = Robot.vision.steeringAdjust();
    rightSpeed =  joystickSpeed -= visionCorrect;
    return rightSpeed;
  }

}