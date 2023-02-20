// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.ArmExtensionSubsystem;

public class ArmOutCMD extends CommandBase {
  /** Creates a new ArmStopCMD. */
  ArmExtensionSubsystem m_ArmExtensionSubsystem;
  public ArmOutCMD(ArmExtensionSubsystem param_ArmExtensionSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_ArmExtensionSubsystem = param_ArmExtensionSubsystem;
    addRequirements(m_ArmExtensionSubsystem);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_ArmExtensionSubsystem.ExtArmOut();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
