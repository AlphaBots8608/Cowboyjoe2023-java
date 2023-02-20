// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDRIVEJoystickPort = 0;
    public static final int kDRIVEforwardReverseAxis = 0;
    public static final int kDRIVELeftRightAxis = 1;
    public static final int klassoMotorAxis = 3;

    public static final int kArmupButton = 8;
    public static final int kArmdownButton = 6;
    public static final int kArmoutButton = 7;
    public static final int kArminButton = 5; 

    public static final int kresetLassoEncoderButton = 9;//#9 is the "select" button in the middle of the joystick
    
    public static final int RatchetLinearActuatorsparkMaxCanID = 15;
  }
  public static class DriveConstants {
    public static final int kFrontLeftSparkMaxCanID = 8;
    public static final int kFollowerLeftSparkMaxCanID = 10;
    public static final int kFrontRightSparkMaxCanID = 6;
    public static final int kFollowerRightSparkMaxCanID = 9;
    public static final boolean kleftInverted = true;
    public static final boolean krightInverted = true;
    
  }

  public static class ArmLifterConstants {
    public static final int kArmLifterSparkMaxCanID = 7;

    public static final double kminEncoderValue = 0; // this should when the arm is vertical and slightly leaning back if not straight.
    public static final double kmaxEncoderValue = 82;// this is when the arm is horizontal and the arm is extended all the way out.

    public static final double kArmLifterUpSpeed = 0.5;
    public static final double kArmLifterDownSpeed = -0.5;
    
    public static int kslewrate = 10;//will be the input slew rate for the arm lifter motor
  }

  public static class ArmExtensionConstants {
    public static final int kArmExtensionSparkMaxCanID = 13;
    
    //double CubeSpeed = .25;
    //double ConeSpeed = .75;

    public static final double kminEncoderValue = 0;
    public static final double kmaxEncoderValue = 320;

    public static final double kArmOutSpeed = 0.75;
    public static final double kArmInSpeed = -0.75;

    public static final int kArmExtensionSlewRate = 10;
  }

  public static class LassoConstants {
    public static final int klassoMotorCanID = 16;
    
    public static final double kCubeSpeed = .25;
    public static final double kConeSpeed = .75;

    //
    //double lassoinchesperrev = 1.5;
    //double lassoencodercountsperrev = 42;
    //double lassoencodercountsperinch = lassoencodercountsperrev/lassoinchesperrev;

   
    public static final double kminEncoderValue = 0;
    public static final double kminEncoderValueWithCube = 96;
    public static final double kmaxEncoderValue = 180;

  
  }
}
