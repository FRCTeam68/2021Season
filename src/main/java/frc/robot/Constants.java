package frc.robot;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    // Controller Inputs

    public static final int XBOX_DRIVE = 0;

    public static final int XBOX_DRIVE_X = 2;
    public static final int XBOX_DRIVE_CIRCLE = 3;
    public static final int XBOX_DRIVE_SQUARE = 1;
    public static final int XBOX_DRIVE_TRIANGLE = 4;
    public static final int XBOX_DRIVE_LY = 1; // left joystick
    public static final int XBOX_DRIVE_LT = 3;
    public static final int XBOX_DRIVE_LT_BUTTON = 7;
    public static final int XBOX_DRIVE_RT = 4;
    public static final int XBOX_DRIVE_RT_BUTTON = 8;
    public static final int XBOX_DRIVE_RY = 5; // right joystick
    public static final int XBOX_DRIVE_SL = 11;
    public static final int XBOX_DRIVE_SR = 12;
    public static final int XBOX_DRIVE_RB = 6;
    public static final int XBOX_DRIVE_LB = 5;
    public static final int XBOX_DRIVE_SHARE = 9;
    public static final int XBOX_DRIVE_OPTIONS = 10;
    public static final int XBOX_DRIVE_POV_DOWN = 180;
    public static final int XBOX_DRIVE_POV_RIGHT = 90;
    public static final int XBOX_DRIVE_POV_LEFT = 270;
    public static final int XBOX_DRIVE_POV_UP = 0;

    //Swerve Module FL
    public static final int kTalonDriveFL = 3;
    public static final int kTalonTurnFL = 4;
    public static final int kCanCoderFL = 11;
    //Swerve Module FR
    public static final int kTalonDriveFR = 7;
    public static final int kTalonTurnFR = 8;
    public static final int kCanCoderFR = 13;
    //Swerve Module BL
    public static final int kTalonDriveBL = 1;
    public static final int kTalonTurnBL = 2;
    public static final int kCanCoderBL = 10;
    //Swerve Module BR
    public static final int kTalonDriveBR = 5;
    public static final int kTalonTurnBR = 6;
    public static final int kCanCoderBR = 12;
    //Swerver Constants
    public static final double kMaxSpeed = Units.feetToMeters(13.6); // 13.6 feet per second
    public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second
    public static final double kDriveP = 15.0;
    public static final double kDriveI = 0.01;
    public static final double kDriveD = 0.1;
    public static final double kDriveF = 0.2;
  
    public static final double kAngleP = 1.0;
    public static final double kAngleI = 0.0;
    public static final double kAngleD = 0.0;
}
