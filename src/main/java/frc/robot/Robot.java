// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
/**
 * This is a demo program showing the use of the DifferentialDrive class. Runs the motors with split
 * arcade steering and an Xbox controller.
 */
public class Robot extends TimedRobot {

  private final PowerDistribution m_pdp = new PowerDistribution();
  
  int frontLeftSparkMaxCanID = 8;
  int followerLeftSparkMaxCanID = 10;
  int frontRightSparkMaxCanID = 6;
  int followerRightSparkMaxCanID = 9;

  int sparkMaxAux1CanID = 15;
  int lassoMotorCanID = 16;
  int sparkMaxAux3CanID = 7;

  private final CANSparkMax m_leftMotor = new CANSparkMax(frontLeftSparkMaxCanID,MotorType.kBrushed);
  private final CANSparkMax m_leftFollowerMotor = new CANSparkMax(followerLeftSparkMaxCanID,MotorType.kBrushed);
  private final CANSparkMax m_rightMotor = new CANSparkMax(frontRightSparkMaxCanID,MotorType.kBrushed);
  private final CANSparkMax m_rightFollowerMotor = new CANSparkMax(followerRightSparkMaxCanID,MotorType.kBrushed);
  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);
  private final XboxController m_driverController = new XboxController(0);
  
  private final CANSparkMax lassoMotor = new CANSparkMax(lassoMotorCanID,MotorType.kBrushless);
  private RelativeEncoder lassoMotor_encoder; 

  //private final CANSparkMax armLiftMotor = new CANSparkMax(sparkMaxAux3CanID,MotorType.kBrushless);
  //private final CANSparkMax armExtensionMotor = new CANSparkMax(sparkMaxAux1CanID,MotorType.kBrushless);
  
  
  @Override
  public void robotInit() {
    SmartDashboard.putData("PDP", m_pdp);
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightMotor.setInverted(true);

    m_leftFollowerMotor.follow(m_leftMotor);
    m_rightFollowerMotor.follow(m_rightMotor);

        /**
    * In order to read encoder values an encoder object is created using the 
    * getEncoder() method from an existing CANSparkMax object
    */
    lassoMotor_encoder = lassoMotor.getEncoder();
  }

  @Override
  public void teleopPeriodic() {
    // Drive with split arcade drive.
    // That means that the Y axis of the left stick moves forward
    // and backward, and the X of the right stick turns left and right.
    m_robotDrive.arcadeDrive(m_driverController.getLeftY(), m_driverController.getLeftX());

    ////////////////////////////////////////////
    lassoMotor.set(m_driverController.getRawAxis(3));
    ///////////////////////
  }

  @Override
  public void robotPeriodic() {

    /**
     * Encoder position is read from a RelativeEncoder object by calling the
     * GetPosition() method.
     * 
     * GetPosition() returns the position of the encoder in units of revolutions
     */
    SmartDashboard.putNumber("Encoder Position", lassoMotor_encoder.getPosition());

    /**
     * Encoder velocity is read from a RelativeEncoder object by calling the
     * GetVelocity() method.
     * 
     * GetVelocity() returns the velocity of the encoder in units of RPM
     */
    SmartDashboard.putNumber("Encoder Velocity", lassoMotor_encoder.getVelocity());

    /////////////////////////////////

    // Get the current going through channel 7, in Amperes.
    // The PDP returns the current in increments of 0.125A.
    // At low currents the current readings tend to be less accurate.
    double current7 = m_pdp.getCurrent(7);
    SmartDashboard.putNumber("Current Channel 7", current7);

    // Get the voltage going into the PDP, in Volts.
    // The PDP returns the voltage in increments of 0.05 Volts.
    double voltage = m_pdp.getVoltage();
    SmartDashboard.putNumber("Voltage", voltage);

    // Retrieves the temperature of the PDP, in degrees Celsius.
    double temperatureCelsius = m_pdp.getTemperature();
    SmartDashboard.putNumber("Temperature", temperatureCelsius);

    // Get the total current of all channels.
    //double totalCurrent = m_pdp.getTotalCurrent();
    //SmartDashboard.putNumber("Total Current", totalCurrent);

    // Get the total power of all channels.
    // Power is the bus voltage multiplied by the current with the units Watts.
    //double totalPower = m_pdp.getTotalPower();
    //SmartDashboard.putNumber("Total Power", totalPower);

    // Get the total energy of all channels.
    // Energy is the power summed over time with units Joules.
    //double totalEnergy = m_pdp.getTotalEnergy();
    //SmartDashboard.putNumber("Total Energy", totalEnergy);
  }
}
