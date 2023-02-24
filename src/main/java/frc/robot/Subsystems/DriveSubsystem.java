package frc.robot.Subsystems;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

//imports for moving motors with spark max motor controllers
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class DriveSubsystem extends SubsystemBase {


    private CANSparkMax m_leftMotor;
    private CANSparkMax m_leftFollowerMotor;
    private CANSparkMax m_rightMotor;
    private CANSparkMax m_rightFollowerMotor;
    private DifferentialDrive m_robotDrive;
    private boolean turbomode = false;

    // private Encoder leftEncoder = new Encoder(//
    //         DriveConstants.kLeftEncoderChannelA, DriveConstants.kLeftEncoderChannelB);
    // private Encoder rightEncoder = new Encoder(//
    //         DriveConstants.kRightEncoderChannelA, DriveConstants.kRightEncoderChannelB);

    public double getEncoderMeters() {
        return 0;
        //return (leftEncoder.get() + -rightEncoder.get()) / 2 * DriveConstants.kEncoderTick2Meter;
    }

    public DriveSubsystem() {
        m_leftMotor = new CANSparkMax(Constants.DriveConstants.kFrontLeftSparkMaxCanID,MotorType.kBrushed);
        m_leftFollowerMotor = new CANSparkMax(Constants.DriveConstants.kFollowerLeftSparkMaxCanID,MotorType.kBrushed);

        m_rightMotor = new CANSparkMax(Constants.DriveConstants.kFrontRightSparkMaxCanID,MotorType.kBrushed);
        m_rightFollowerMotor = new CANSparkMax(Constants.DriveConstants.kFollowerRightSparkMaxCanID,MotorType.kBrushed);
        
        m_robotDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);
        
        // We need to invert one side of the drivetrain so that positive voltages
        // result in both sides moving forward. Depending on how your robot's
        // gearbox is constructed, you might have to invert the left side instead.
        m_leftMotor.setInverted(Constants.DriveConstants.kleftInverted);
        m_rightMotor.setInverted(Constants.DriveConstants.krightInverted);

        //set follower motors
        m_leftFollowerMotor.follow(m_leftMotor);
        m_rightFollowerMotor.follow(m_rightMotor);

        //feed forward is currently unused
        double kSCimMotor = 0;
        double kVCimMotor = 0;
        double kACimMotor = 0;
        SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kSCimMotor, kVCimMotor);
    }

    @Override
    public void periodic() {
        //SmartDashboard.putNumber("Drive encoder value", getEncoderMeters());
    }
    public void changeturbomode(){
        turbomode = !turbomode;
    }

    public void setMotors(double leftSpeed, double rightSpeed) {
        //driveLeftMotor.set(leftSpeed);
        //driveRightMotor.set(-rightSpeed);
    }
    public void ArcadeDrivemotors(double XSpeed, double ZRotation) {
        // Drive with split arcade drive.
        // That means that the Y axis of the left stick moves forward
        // and backward, and the X of the right stick turns left and right.
        if(!turbomode)
        {
            XSpeed = XSpeed/1.75;
            ZRotation = ZRotation/1.5;
        }
        else
        {
            ZRotation = ZRotation/1.2;
        }
        
        m_robotDrive.arcadeDrive(XSpeed, ZRotation);
    }

}

