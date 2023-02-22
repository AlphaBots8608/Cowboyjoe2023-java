// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ResourceBundle.Control;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight3Subsystem extends SubsystemBase {
  /** Creates a new Limelight3Subsystem. */
  public Limelight3Subsystem(XboxController controllerUsedToScore) {
    this.ControllerUsedToScore = controllerUsedToScore;
  }

  XboxController ControllerUsedToScore;
  NetworkTable latestInfo;
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    getdataToDashboard();
    VibeOnZero();
    
  }
  public void VibeOnZero() {
    
    NetworkTableEntry tx = latestInfo.getEntry("tx");
    NetworkTableEntry ty = latestInfo.getEntry("ty");
    NetworkTableEntry ta = latestInfo.getEntry("ta");

    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);
    
    double kXClosenessForRumble = 5.0;
    double kYClosenessForRumble = 1.0;
    double kTagAreaofScreenBeforeRumbleOn = .4;
    if(area > kTagAreaofScreenBeforeRumbleOn)
    {
      if(Math.abs(x) < kXClosenessForRumble && Math.abs(x) > 0.0)
      {
        ControllerUsedToScore.setRumble(RumbleType.kRightRumble, .3);
      }
      if(Math.abs(y) < kYClosenessForRumble && Math.abs(y) > 0.0)
      {
        ControllerUsedToScore.setRumble(RumbleType.kLeftRumble, .2);
      }
    } 
    else 
    {
      ControllerUsedToScore.setRumble(RumbleType.kBothRumble, 0);
    }
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
  }
  public void getdataToDashboard()
  {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    latestInfo = table;

    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");

    //read values periodically
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);

    //post to smart dashboard periodically
    //SmartDashboard.putNumber("LimelightX", x);
    //SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
  }
}
