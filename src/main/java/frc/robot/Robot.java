// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//imports for commands
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import edu.wpi.first.wpilibj.TimedRobot;

import javax.lang.model.util.ElementScanner14;

//imports for lasso control/command and encoders
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This is a demo program showing the use of the DifferentialDrive class. Runs the motors with split
 * arcade steering and an Xbox controller.
 */
public class Robot extends TimedRobot {

  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;


  JoeColorSensor CSensor= new JoeColorSensor();
  JoePowerDistributionPanel PDP= new JoePowerDistributionPanel();
  
  //private final XboxController m_driverController = new XboxController(0);

  
  int lassoMotorCanID = 16;
  private final CANSparkMax lassoMotor = new CANSparkMax(lassoMotorCanID,MotorType.kBrushless);
  private RelativeEncoder lassoMotor_encoder; 

  //private final CANSparkMax armLiftMotor = new CANSparkMax(sparkMaxAux3CanID,MotorType.kBrushless);
  //private final CANSparkMax armExtensionMotor = new CANSparkMax(sparkMaxAux1CanID,MotorType.kBrushless);
  
  


  
  

  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    
 

        /**
    * In order to read encoder values an encoder object is created using the 
    * getEncoder() method from an existing CANSparkMax object
    */
    lassoMotor_encoder = lassoMotor.getEncoder();
  }

  @Override
  public void robotPeriodic() {
    getEncoderData();
    PDP.GetPdpData();
    CSensor.GetColorSensorData();

    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

  }

    /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
    @Override
    public void autonomousInit() {
      m_autonomousCommand = m_robotContainer.getAutonomousCommand();
  
      // schedule the autonomous command (example)
      if (m_autonomousCommand != null) {
        m_autonomousCommand.schedule();
      }
    }
  
    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {}



    @Override
    public void teleopInit() {
      // This makes sure that the autonomous stops running when
      // teleop starts running. If you want the autonomous to
      // continue until interrupted by another command, remove
      // this line or comment it out.
      if (m_autonomousCommand != null) {
        m_autonomousCommand.cancel();
      }
    }
    @Override
    public void teleopPeriodic() {

  
      ////////////////////////////////////////////
      double lassospeed = 0;
      if (CSensor.lastdetectedColor == "Cube")
      {
        lassospeed = m_robotContainer.joystick1.getRawAxis(3)/2;
      }
      else if (CSensor.lastdetectedColor == "Cone")
      {
        lassospeed = m_robotContainer.joystick1.getRawAxis(3);
      }
      else 
      {
        lassospeed = m_robotContainer.joystick1.getRawAxis(3)/4;
      }
      lassoMotor.set(lassospeed);
      ///////////////////////
    }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  public void getEncoderData()
  {
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

  }
  
}
