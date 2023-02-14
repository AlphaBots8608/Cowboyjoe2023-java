// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/**
 * This is a demo program showing the use of the DifferentialDrive class. Runs the motors with split
 * arcade steering and an Xbox controller.
 */
public class Robot extends TimedRobot {

  int frontLeftSparkMaxCanID = 8;
  int followerLeftSparkMaxCanID = 10;
  int frontRightSparkMaxCanID = 6;
  int followerRightSparkMaxCanID = 9;

  int sparkMaxAux1CanID = 15;
  int sparkMaxAux2CanID = 14;

  private final CANSparkMax m_leftMotor = new CANSparkMax(frontLeftSparkMaxCanID,MotorType.kBrushed);
  private final CANSparkMax m_leftFollowerMotor = new CANSparkMax(followerLeftSparkMaxCanID,MotorType.kBrushed);
  private final CANSparkMax m_rightMotor = new CANSparkMax(frontRightSparkMaxCanID,MotorType.kBrushed);
  private final CANSparkMax m_rightFollowerMotor = new CANSparkMax(followerRightSparkMaxCanID,MotorType.kBrushed);
  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);
  private final XboxController m_driverController = new XboxController(0);
 

  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightMotor.setInverted(true);

    m_leftFollowerMotor.follow(m_leftMotor);
    m_rightFollowerMotor.follow(m_rightMotor);
  }

  @Override
  public void teleopPeriodic() {
    // Drive with split arcade drive.
    // That means that the Y axis of the left stick moves forward
    // and backward, and the X of the right stick turns left and right.
    m_robotDrive.arcadeDrive(m_driverController.getLeftY(), m_driverController.getLeftX());
  }
}
