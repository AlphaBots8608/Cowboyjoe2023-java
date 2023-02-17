package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


public class RobotContainer {
    //CONFIG
    int DRIVEJoystickPort = 0;
    int DRIVEforwardReverseAxis = 0;
    int DRIVELeftRightAxis = 1;

    //
    
    
    int RatchetLinearActuatorsparkMaxCanID = 15;
    

    public JoeColorSensor CSensor= new JoeColorSensor();
    public JoePowerDistributionPanel PDP= new JoePowerDistributionPanel();

    private final DriveSubsystem driveSubsystem = new DriveSubsystem();
    // private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    // private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    public LassoSubsystem LassoSubsystem = new LassoSubsystem();
    public ArmExtensionSubsystem ArmExtensionSubsystem = new ArmExtensionSubsystem();
    public ArmLifterSubsystem ArmLifterSubsystem = new ArmLifterSubsystem();

    public final Joystick joystick1 = new Joystick(DRIVEJoystickPort);

    public RobotContainer() {
        configureButtonBindings();

        driveSubsystem.setDefaultCommand(new ArcadeDriveCmd(driveSubsystem, //
                () -> joystick1.getRawAxis(DRIVEforwardReverseAxis),
                () -> joystick1.getRawAxis(DRIVELeftRightAxis))//
        );
        // elevatorSubsystem.setDefaultCommand(new ElevatorJoystickCmd(elevatorSubsystem, 0));
        // intakeSubsystem.setDefaultCommand(new IntakeSetCmd(intakeSubsystem, true));
        //LassoSubsystem.setDefaultCommand(new LassoJoystickCmd(LassoSubsystem,CSensor,()->joystick1.getRawAxis(3)));
        //ArmExtensionSubsystem.setDefaultCommand(new ArmExtensionJoystickCmd(ArmExtensionSubsystem,()->joystick1.getRawAxis(3)));
        ArmLifterSubsystem.setDefaultCommand(new ArmLifterJoystickCmd(ArmLifterSubsystem,()->joystick1.getRawAxis(3)));
    }

    private void configureButtonBindings() {
        // new JoystickButton(joystick1, OIConstants.kElevatorPIDRaiseButtonIdx)
        //         .whileActiveOnce(new ElevatorPIDCmd(elevatorSubsystem, ElevatorConstants.kRaisedPosition));
        // new JoystickButton(joystick1, OIConstants.kElevatorPIDLowerButtonIdx)
        //         .whileActiveOnce(new ElevatorPIDCmd(elevatorSubsystem, ElevatorConstants.kLoweredPosition));
        // new JoystickButton(joystick1, OIConstants.kElevatorJoystickRaiseButtonIdx)
        //         .whileActiveOnce(new ElevatorJoystickCmd(elevatorSubsystem, ElevatorConstants.kJoystickMaxSpeed));
        // new JoystickButton(joystick1, OIConstants.kElevatorJoystickLowerButtonIdx)
        //         .whileActiveOnce(new ElevatorJoystickCmd(elevatorSubsystem, -ElevatorConstants.kJoystickMaxSpeed));
        // new JoystickButton(joystick1, OIConstants.kIntakeCloseButtonIdx)
        //         .whileActiveOnce(new IntakeSetCmd(intakeSubsystem, false));
    }

    public Command getAutonomousCommand() {
        return new SequentialCommandGroup();
        // return new SequentialCommandGroup( //
        //         new DriveForwardCmd(driveSubsystem, DriveConstants.kAutoDriveForwardDistance), //
        //         new ParallelCommandGroup( //
        //                 new IntakeSetCmd(intakeSubsystem, false), //
        //                 new ElevatorPIDCmd(elevatorSubsystem, ElevatorConstants.kRaisedPosition) //
        //         )//
        // );
    }
}