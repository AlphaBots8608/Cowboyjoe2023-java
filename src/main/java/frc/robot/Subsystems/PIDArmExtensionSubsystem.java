package frc.robot.Subsystems;


import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class PIDArmExtensionSubsystem extends PIDSubsystem {

  static double kP = 0.05;
  static double kI = 0.0;
  static double kD = 0.0;

    double extensionMotorEncoderValue = 0;
    double extensionMotorEncoderVelocity = 0;
    
    SlewRateLimiter speedLimiter = new SlewRateLimiter(Constants.ArmExtensionConstants.kArmExtensionSlewRate);
    private final CANSparkMax extensionMotor = new CANSparkMax(Constants.ArmExtensionConstants.kArmExtensionSparkMaxCanID,MotorType.kBrushless);
    private RelativeEncoder extensionMotor_encoder; 

    public PIDArmExtensionSubsystem() {
      super(new PIDController(kP, kI, kD));
      setSetpoint(0);
      extensionMotor_encoder = extensionMotor.getEncoder();

      extensionMotor_encoder.setPosition(0);
      extensionMotor.setInverted(false);
      //lassoMotor_encoder.setVelocityConversionFactor(lassoencodercountsperinch);
      extensionMotor.setSoftLimit(SoftLimitDirection.kForward, (float)Constants.ArmExtensionConstants.kmaxEncoderValue);
      extensionMotor.setSoftLimit(SoftLimitDirection.kReverse, (float)Constants.ArmExtensionConstants.kminEncoderValue);
      extensionMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
      extensionMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);

    }

  @Override
  public double getMeasurement() {
    extensionMotorEncoderValue = extensionMotor_encoder.getPosition();
    SmartDashboard.putNumber("ArmExtension PID Encoder Position",extensionMotorEncoderValue);
    return extensionMotorEncoderValue;
  }

  @Override
  public void useOutput(double output, double setpoint) {
    SetSpeed(output);
    SmartDashboard.putNumber("ArmExtension PID output",output);
    SmartDashboard.putNumber("ArmExtension SetPoint",setpoint);
    //m_shooterMotor.setVoltage(output + m_shooterFeedforward.calculate(setpoint));
  }

    @Override
    public void periodic() {
      getEncoderData();
      super.periodic();
    }


  public void setSetpointHighestScore() {
    setSetpoint(320);
  }
  public void setSetpointMidScore() {
    setSetpoint(250);
  }
  public void setSetpointLowScore() {
    setSetpoint(60);
  }
  public void setSetpointIn() {
    setSetpoint(0);
  }
    public void getEncoderData()
  {
    /**
     * Encoder position is read from a RelativeEncoder object by calling the
     * GetPosition() method.
     * 
     * GetPosition() returns the position of the encoder in units of revolutions
     */
    //extensionMotorEncoderValue = extensionMotor_encoder.getPosition();
    //SmartDashboard.putNumber("ArmExtension Encoder Position",extensionMotorEncoderValue);

    /**
     * Encoder velocity is read from a RelativeEncoder object by calling the
     * GetVelocity() method.
     * 
     * GetVelocity() returns the velocity of the encoder in units of RPM
     */
    extensionMotorEncoderVelocity = extensionMotor_encoder.getVelocity();
    SmartDashboard.putNumber("ArmExtension Encoder Velocity", extensionMotorEncoderVelocity);

  }
  public void ExtArmOut() {
    SetSpeed(speedLimiter.calculate(Constants.ArmExtensionConstants.kArmOutSpeed));
  }
  public void ExtArmIn() {
    SetSpeed(speedLimiter.calculate(Constants.ArmExtensionConstants.kArmInSpeed));
  }
  public void ExtArmStop() {
    SetSpeed(0);
  }


  public void SetSpeed(double thisspeed) {
    extensionMotor.set(thisspeed);
  } 
}