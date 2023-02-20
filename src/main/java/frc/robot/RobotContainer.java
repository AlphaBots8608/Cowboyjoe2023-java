package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Commands.ArcadeDriveCmd;
import frc.robot.Commands.ArmDownCMD;
import frc.robot.Commands.ArmExtStopCMD;
import frc.robot.Commands.ArmInCMD;
import frc.robot.Commands.ArmOutCMD;
import frc.robot.Commands.ArmStopCMD;
import frc.robot.Commands.ArmUpCMD;
import frc.robot.Commands.LassoJoystickCmd;
import frc.robot.Subsystems.ArmExtensionSubsystem;
import frc.robot.Subsystems.ArmLifterSubsystem;
import frc.robot.Subsystems.DriveSubsystem;
import frc.robot.Subsystems.JoeColorSensor;
import frc.robot.Subsystems.JoePowerDistributionPanel;
import frc.robot.Subsystems.LassoSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;



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
        //intakeSubsystem.setDefaultCommand(new IntakeSetCmd(intakeSubsystem, true));
        LassoSubsystem.setDefaultCommand(new LassoJoystickCmd(LassoSubsystem,CSensor,()->joystick1.getRawAxis(3)));
        //ArmExtensionSubsystem.setDefaultCommand(new ArmExtensionJoystickCmd(ArmExtensionSubsystem,()->joystick1.getRawAxis(2)));
        ArmExtensionSubsystem.setDefaultCommand(new ArmExtStopCMD(ArmExtensionSubsystem));
        ArmLifterSubsystem.setDefaultCommand(new ArmStopCMD(ArmLifterSubsystem));
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
        JoystickButton armup = new JoystickButton(joystick1, 8);
        JoystickButton armdown = new JoystickButton(joystick1, 6);
        JoystickButton armout = new JoystickButton(joystick1, 7);
        JoystickButton armin = new JoystickButton(joystick1, 5);

        armup.whileTrue(new ArmUpCMD(ArmLifterSubsystem));
        armdown.whileTrue(new ArmDownCMD(ArmLifterSubsystem));
        armout.whileTrue(new ArmOutCMD(ArmExtensionSubsystem));
        armin.whileTrue(new ArmInCMD(ArmExtensionSubsystem));

        new JoystickButton(joystick1, 9).onTrue(new InstantCommand(ArmLifterSubsystem::resetEncoder));
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