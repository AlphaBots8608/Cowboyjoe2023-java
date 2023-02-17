package frc.robot;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class ArmLifterSubsystem extends SubsystemBase {

  int ArmLifterSparkMaxCanID = 7;
  
  double minEncoderValue = 0;
  double maxEncoderValue = 90;

  double EncoderValue = 0;
  double EncoderVelocity = 0;
  private final CANSparkMax lifterMotor = new CANSparkMax(ArmLifterSparkMaxCanID,MotorType.kBrushless);
  private RelativeEncoder lifterMotor_encoder; 

    public ArmLifterSubsystem() {
      lifterMotor_encoder = lifterMotor.getEncoder();

      lifterMotor_encoder.setPosition(0);
      lifterMotor.setInverted(true);
      //lassoMotor_encoder.setVelocityConversionFactor(lassoencodercountsperinch);
      lifterMotor.setSoftLimit(SoftLimitDirection.kForward, (float)maxEncoderValue);
      lifterMotor.setSoftLimit(SoftLimitDirection.kReverse, (float)minEncoderValue);
      lifterMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
      lifterMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);

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
    EncoderValue = lifterMotor_encoder.getPosition();
    SmartDashboard.putNumber("ArmLifter Encoder Position",EncoderValue);

    /**
     * Encoder velocity is read from a RelativeEncoder object by calling the
     * GetVelocity() method.
     * 
     * GetVelocity() returns the velocity of the encoder in units of RPM
     */
    EncoderVelocity = lifterMotor_encoder.getVelocity();
    SmartDashboard.putNumber("ArmLifter Encoder Velocity", EncoderVelocity);

  }
  /**
   * 
   * feed this a color sensor and speed from an axis of -1 to 1
   */
  public void SetSpeed(double thisspeed) {
    lifterMotor.set(-thisspeed);
  }
}