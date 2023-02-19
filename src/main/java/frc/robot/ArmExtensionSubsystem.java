package frc.robot;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class ArmExtensionSubsystem extends SubsystemBase {

    int ArmExtensionSparkMaxCanID = 13;
    
    //double CubeSpeed = .25;
    //double ConeSpeed = .75;

    double minEncoderValue = 0;
    double maxEncoderValue = 320;

    double extensionMotorEncoderValue = 0;
    double extensionMotorEncoderVelocity = 0;
    private final CANSparkMax extensionMotor = new CANSparkMax(ArmExtensionSparkMaxCanID,MotorType.kBrushless);
    private RelativeEncoder extensionMotor_encoder; 

    public ArmExtensionSubsystem() {
      extensionMotor_encoder = extensionMotor.getEncoder();

      extensionMotor_encoder.setPosition(0);
      extensionMotor.setInverted(false);
      //lassoMotor_encoder.setVelocityConversionFactor(lassoencodercountsperinch);
      extensionMotor.setSoftLimit(SoftLimitDirection.kForward, (float)maxEncoderValue);
      extensionMotor.setSoftLimit(SoftLimitDirection.kReverse, (float)minEncoderValue);
      extensionMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
      extensionMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);

    }

    @Override
    public void periodic() {
      getEncoderData();
    }


    public void getEncoderData()
  {
    /**
     * Encoder position is read from a RelativeEncoder object by calling the
     * GetPosition() method.
     * 
     * GetPosition() returns the position of the encoder in units of revolutions
     */
    extensionMotorEncoderValue = extensionMotor_encoder.getPosition();
    SmartDashboard.putNumber("ArmExtension Encoder Position",extensionMotorEncoderValue);

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
    SetSpeed(.75);
  }
  public void ExtArmIn() {
    SetSpeed(-.75);
  }
  public void ExtArmStop() {
    SetSpeed(0);
  }


  public void SetSpeed(double thisspeed) {
    extensionMotor.set(thisspeed);
  } 
}