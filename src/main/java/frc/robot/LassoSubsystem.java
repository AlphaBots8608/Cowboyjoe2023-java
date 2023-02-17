package frc.robot;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class LassoSubsystem extends SubsystemBase {

    int lassoMotorCanID = 16;
    
    double CubeSpeed = .25;
    double ConeSpeed = .75;

    //
    
    private final CANSparkMax lassoMotor = new CANSparkMax(lassoMotorCanID,MotorType.kBrushless);
    private RelativeEncoder lassoMotor_encoder; 

    public LassoSubsystem() {
        lassoMotor_encoder = lassoMotor.getEncoder();
    }

    @Override
    public void periodic() {
    }


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
  /**
   * 
   * feed this a color sensor and speed from an axis of -1 to 1
   */
  public void SetlassoSpeed(JoeColorSensor thisSensor,double thisspeed) {
    double lassospeed = 0;
    if (thisSensor.lastdetectedColor == "Cube")
    {
      lassospeed = thisspeed/2;
    }
    else if (thisSensor.lastdetectedColor == "Cone")
    {
      lassospeed = thisspeed;
    }
    else 
    {
      lassospeed = thisspeed/4;
    }
    lassoMotor.set(lassospeed);
  }
}