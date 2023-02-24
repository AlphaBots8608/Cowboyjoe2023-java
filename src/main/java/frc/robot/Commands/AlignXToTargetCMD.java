// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableType;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.ArmLifterSubsystem;
import frc.robot.Subsystems.DriveSubsystem;
import frc.robot.Subsystems.Limelight3Subsystem;


public class AlignXToTargetCMD extends CommandBase {
  /** Creates a new ArmStopCMD. */
  DriveSubsystem driveSubsystem;
  Limelight3Subsystem limelight3Subsystem;
  public AlignXToTargetCMD(DriveSubsystem driveSubsystem,Limelight3Subsystem ThisLimelight) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveSubsystem = driveSubsystem;
    limelight3Subsystem = ThisLimelight;
    addRequirements(driveSubsystem);
    addRequirements(limelight3Subsystem);
  }

  PIDController AligXController;// this will turn left or right to align
  PIDController AlignZController;//this will go forward and back to align. 
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    double kXP = 0.06;
    double kXI = 0.01;
    double kXD = 0.01;
       
    AligXController =  new PIDController(kXP,kXI,kXD);

    double kZP = 1.0;
    double kZI = 0.0001;
    double kZD = 0.0001;
       
    AlignZController =  new PIDController(kZP,kZI,kZD);

    
  }
  int pipeline = 0;
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //double KpAim = -0.1f;
    //double KpDistance = -0.1f;
    double min_aim_command = 0.15;
    double min_forward_command = 0.1f;

    double x = limelight3Subsystem.getXOffset(); 
    double y = limelight3Subsystem.getYOffset();
    double area = limelight3Subsystem.getArea();

    double targetOffsetAngle_Vertical = y;


    // how many degrees back is your limelight rotated from perfectly vertical?
    double limelightMountAngleDegrees = 0.0;

    // distance from the center of the Limelight lens to the floor
    double limelightLensHeightInches = 17.0;

    // distance from the target to the floor
    double goalHeightInches = 11.0;

    double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
    double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

    //calculate distance
    double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches)/Math.tan(angleToGoalRadians);

    double heading_error = x * -1.0;
    double distance_error = y * -1.0;
    double tagArea_error = area;
    double tagArea_setpoint = 1.01;
    double steering_adjust = 0.0;
    double distance_adjust =  0.0;
    if (Math.abs(heading_error) > .0)
    {
      if (heading_error < 0)
      {
              steering_adjust = AligXController.calculate(heading_error,0) + min_aim_command;
      }
      else
      {
              steering_adjust = AligXController.calculate(heading_error,0) - min_aim_command;
      }
    }
    double targetMustBeThisSizeBeforeAutomating = .4;
    if (Math.abs(tagArea_error) > targetMustBeThisSizeBeforeAutomating)
    {
      distance_adjust = AlignZController.calculate(tagArea_error,tagArea_setpoint); //KpDistance * distance_error;/// we are ignoring Y (up down) for now. 

      if (heading_error < 0)
      {
        distance_adjust += min_forward_command;
      }
      else
      {
        distance_adjust -= min_forward_command; //KpDistance * distance_error;/// we are ignoring Y (up down) for now. 
      }
    }
    // double left_command = steering_adjust + distance_adjust;
    // double right_command = steering_adjust + distance_adjust;
    driveSubsystem.ArcadeDrivemotors(0, steering_adjust);
    SmartDashboard.putNumber("distance_adjust", distance_adjust);
    SmartDashboard.putNumber("steering_adjust", steering_adjust);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
