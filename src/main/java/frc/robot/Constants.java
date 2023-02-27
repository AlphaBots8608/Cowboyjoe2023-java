// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class XboxControllerMap {
    public static final int kLeftStickUpDownAxis = 1;
    public static final int kLeftStickLeftRightAxis = 0;
    public static final int kLeftStickButton = 9;

    public static final int kRightStickUpDownAxis = 5;
    public static final int kRightStickLeftRightAxis = 4;
    public static final int kRightStickButton = 10;

    public static final int kLeftTriggerAxis = 2;
    public static final int kRightTriggerAxis = 3;

    public static final int kLeftBumperButton = 5;
    public static final int kRightBumperButton = 6;

     
    public static final int kYButton = 4;// Y button, Triangle button or the Top button of the xbox controller
    public static final int kBButton = 2;// B button, Square button or the Right button of the xbox controller
    public static final int kXButton = 3;// X button, Left button of the xbox controller
    public static final int kAButton = 1;// A button, bottom of 4 buttons

    public static final int kLeftSelectButton = 7;// Middle buttons on the left the View change button the select button on nintendo
    public static final int kRightStartButton = 8;// middle buttons, right side, menu button or start button on nintendo

    //these are the pov directional buttons (this had 8 directions but we do not have them mapped.)
    public static final int kPOVDirectionUP = 0;//0 is up on xbox controller POV hat
    public static final int kPOVDirectionDOWN = 180;//180 is down on xbox controller POV hat
    public static final int kPOVDirectionRIGHT = 90;//90 is right on xbox controller POV hat
    public static final int kPOVDirectionLeft = 270;//270 is Left on xbox controller POV hat

  }
  public static class OperatorConstants {
    public static final int kDRIVEJoystickPort = 0;
    public static final int kDRIVEforwardReverseAxis = XboxControllerMap.kLeftStickUpDownAxis;
    public static final int kDRIVELeftRightAxis = XboxControllerMap.kLeftStickLeftRightAxis;
    public static final int klassoMotorAxis = XboxControllerMap.kRightStickUpDownAxis;
    public static final int kDRIVERightTriggerAxis = XboxControllerMap.kRightTriggerAxis;
    public static final int kDRIVELeftTriggerAxis = XboxControllerMap.kLeftTriggerAxis;
    public static final int kArmupButton = 6;
    public static final int kArmdownButton = 8;
    public static final int kArmoutButton = 7;
    public static final int kArminButton = 5; 

    //public static final int kresetLassoEncoderButton = 9;//#9 is the "select" button in the middle of the joystick
    
    public static final int RatchetLinearActuatorsparkMaxCanID = 15;
  }
  public static class DriveConstants {
    public static final int kFrontLeftSparkMaxCanID = 8;
    public static final int kFollowerLeftSparkMaxCanID = 10;
    public static final int kFrontRightSparkMaxCanID = 6;
    public static final int kFollowerRightSparkMaxCanID = 9;
    public static final boolean kleftInverted = false;
    public static final boolean krightInverted = true;
    
  }

  public static class ArmLifterConstants {
    public static final int kArmLifterSparkMaxCanID = 7;

    public static final double kminEncoderValue = 0; // this should when the arm is vertical and slightly leaning back if not straight.
    public static final double kmaxEncoderValue = 65;// this is when the arm is horizontal and the arm is extended all the way out.
    public static final double kmaxEncoderValueCollapsed=65; //this is when the amr is horizontal and the arm is collapsed all the way in
    public static final double kmaxEncoderValueExtended=60; //this is when the amr is horizontal and the arm is collapsed all the way in

    public static final double kArmLifterUpSpeed = 0.5;
    public static final double kArmLifterDownSpeed = -0.5;
    
    public static final int kArmLifterSlewRate =8;
    public static int kslewrate = 10;//will be the input slew rate for the arm lifter motor

    public static final double kGoalScoringEncoderValue = 36.7;

  }

  public static class ArmExtensionConstants {
    public static final int kArmExtensionSparkMaxCanID = 13;
    
    //double CubeSpeed = .25;
    //double ConeSpeed = .75;

    public static final double kminEncoderValue = 0;
    public static final double kmaxEncoderValue = 220;

    public static final double kArmOutSpeed = 0.75;
    public static final double kArmInSpeed = -0.75;

    public static final int kArmExtensionSlewRate = 10;

    public static final double kHighestGoalEncoderValue = 195;
    public static final double kMidestGoalEncoderValue = 30;
    public static final double kLowestGoalEncoderValue = 0;
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
    public static final double kminEncoderValueWithCube = 93;
    public static final double kEncoderValueLoopOut= 150;
    public static final double kmaxEncoderValue = 180;

    public static final double kminEncoderValueWithCone = 4;

  
  }
}
