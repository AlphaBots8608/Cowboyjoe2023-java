package frc.robot;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class LassoSubsystem extends SubsystemBase {

    int lassoMotorCanID = 16;
    
    double CubeSpeed = .25;
    double ConeSpeed = .75;

    //
    //double lassoinchesperrev = 1.5;
    //double lassoencodercountsperrev = 42;
    //double lassoencodercountsperinch = lassoencodercountsperrev/lassoinchesperrev;

    //double Currentlassolength = 0; // this is the current length of the lasso in inches that is 'out' from the motor. 0 is retracted to tightest position (slight slack)
    
    double minEncoderValue = 0;
    double maxEncoderValue = 180;

    double lassoEncoderValue = 0;
    double lassoEncoderVelocity = 0;
    private final CANSparkMax lassoMotor = new CANSparkMax(lassoMotorCanID,MotorType.kBrushless);
    private RelativeEncoder lassoMotor_encoder; 

    public LassoSubsystem() {
        lassoMotor_encoder = lassoMotor.getEncoder();

        lassoMotor_encoder.setPosition(0);
        lassoMotor.setInverted(true);
        //lassoMotor_encoder.setVelocityConversionFactor(lassoencodercountsperinch);
        lassoMotor.setSoftLimit(SoftLimitDirection.kForward, (float)maxEncoderValue);
        lassoMotor.setSoftLimit(SoftLimitDirection.kReverse, (float)minEncoderValue);
        lassoMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
        lassoMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);

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
    lassoEncoderValue = lassoMotor_encoder.getPosition();
    SmartDashboard.putNumber("Lasso Encoder Position",lassoEncoderValue);

    /**
     * Encoder velocity is read from a RelativeEncoder object by calling the
     * GetVelocity() method.
     * 
     * GetVelocity() returns the velocity of the encoder in units of RPM
     */
    lassoEncoderVelocity = lassoMotor_encoder.getVelocity();
    SmartDashboard.putNumber("Lasso Encoder Velocity", lassoEncoderVelocity);

  }
  /**
   * 
   * feed this a color sensor and speed from an axis of -1 to 1
   */
  public void SetlassoSpeed(JoeColorSensor thisSensor,double thisspeed) {
    double lassospeed = 0;



    //final speed divider if you are pulling in a cube. This is a safety feature to prevent the lasso from pulling in the cube too fast and breaking it. 
    // todo make this look at amperage on motor and stop if it is too high (ie the motor is stalled or the lasso is pulled tight enough)
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
      lassospeed = thisspeed;
    }
    lassoMotor.set(lassospeed);
  }
}