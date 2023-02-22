package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.ArcadeDriveCmd;
import frc.robot.Commands.ArmDownCMD;
import frc.robot.Commands.ArmExtStopCMD;
import frc.robot.Commands.ArmInCMD;
import frc.robot.Commands.ArmLifterJoystickCmd;
import frc.robot.Commands.ArmOutCMD;
import frc.robot.Commands.ArmStopCMD;
import frc.robot.Commands.ArmUpCMD;
import frc.robot.Commands.LassoJoystickCmd;
import frc.robot.Subsystems.ArmExtensionSubsystem;
import frc.robot.Subsystems.ArmLifterSubsystem;
import frc.robot.Subsystems.DriveSubsystem;
import frc.robot.Subsystems.JoeColorSensor;
import frc.robot.Subsystems.JoePowerDistributionPanel;
import frc.robot.Subsystems.Limelight3Subsystem;
import frc.robot.Subsystems.PIDLassoSubsystem;
import frc.robot.Subsystems.PIDArmExtensionSubsystem;
import frc.robot.Subsystems.PIDArmLifterSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;



public class RobotContainer {
    public JoePowerDistributionPanel PDP= new JoePowerDistributionPanel();
    public final XboxController driveController = new XboxController(Constants.OperatorConstants.kDRIVEJoystickPort);
    

    private final DriveSubsystem driveSubsystem = new DriveSubsystem();
    public JoeColorSensor CSensor= new JoeColorSensor();
    public Limelight3Subsystem limelight3Subsystem = new Limelight3Subsystem(driveController);
    // private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    // private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    public PIDLassoSubsystem PIDLassoSubsystem = new PIDLassoSubsystem();
    //public ArmExtensionSubsystem ArmExtensionSubsystem = new ArmExtensionSubsystem();
    public PIDArmExtensionSubsystem PIDArmExtensionSubsystem = new PIDArmExtensionSubsystem();
    //public ArmLifterSubsystem ArmLifterSubsystem = new ArmLifterSubsystem();
    public PIDArmLifterSubsystem PIDArmLifterSubsystem = new PIDArmLifterSubsystem();
    //public final Joystick driveController = new Joystick(Constants.OperatorConstants.kDRIVEJoystickPort);
    
    public RobotContainer() {
        configureButtonBindings();
        driveSubsystem.setDefaultCommand(new ArcadeDriveCmd(driveSubsystem, //
                () -> driveController.getRawAxis(Constants.OperatorConstants.kDRIVEforwardReverseAxis),
                () -> driveController.getRawAxis(Constants.OperatorConstants.kDRIVELeftRightAxis))//
        );
        // elevatorSubsystem.setDefaultCommand(new ElevatorJoystickCmd(elevatorSubsystem, 0));
        //intakeSubsystem.setDefaultCommand(new IntakeSetCmd(intakeSubsystem, true));
        
        PIDLassoSubsystem.setDefaultCommand(new LassoJoystickCmd(PIDLassoSubsystem,CSensor,()->driveController.getRawAxis(Constants.OperatorConstants.klassoMotorAxis)));
        //ArmExtensionSubsystem.setDefaultCommand(new ArmExtensionJoystickCmd(ArmExtensionSubsystem,()->joystick1.getRawAxis(2)));
        //ArmExtensionSubsystem.setDefaultCommand(new ArmExtStopCMD(ArmExtensionSubsystem));
        //PIDArmLifterSubsystem.setDefaultCommand(new ArmLifterJoystickCmd(PIDArmLifterSubsystem, () -> driveController.getRawAxis(Constants.OperatorConstants.kDRIVERightTriggerAxis)));
    }

    private void configureButtonBindings() {

        //here is one way to handle trigger/axis inputs that act as a button but accept their analog input.
        //this also has the onFalse for both axis triggers that executes when while true is done handled but the Trigger Class.
        Trigger ArmUpTrigger = new Trigger(()->driveController.getRawAxis(Constants.OperatorConstants.kDRIVERightTriggerAxis) > 0.2);
        ArmUpTrigger.whileTrue(new ArmLifterJoystickCmd(PIDArmLifterSubsystem, () -> driveController.getRawAxis(Constants.OperatorConstants.kDRIVERightTriggerAxis)));
        ArmUpTrigger.onFalse(new InstantCommand(PIDArmLifterSubsystem::setSetpointAtCurrentPoint,PIDArmLifterSubsystem));
        Trigger ArmDownTrigger = new Trigger(()->driveController.getRawAxis(Constants.OperatorConstants.kDRIVELeftTriggerAxis) > 0.2);
        ArmDownTrigger.whileTrue(new ArmLifterJoystickCmd(PIDArmLifterSubsystem, () -> -driveController.getRawAxis(Constants.OperatorConstants.kDRIVELeftTriggerAxis)));
        ArmDownTrigger.onFalse(new InstantCommand(PIDArmLifterSubsystem::setSetpointAtCurrentPoint,PIDArmLifterSubsystem));
        //JoystickButton armup = new JoystickButton(driveController, Constants.OperatorConstants.kArmupButton);
        //JoystickButton armdown = new JoystickButton(driveController, Constants.OperatorConstants.kArmdownButton);
        //JoystickButton armout = new JoystickButton(joystick1, Constants.OperatorConstants.kArmoutButton);
        //JoystickButton armin = new JoystickButton(joystick1, Constants.OperatorConstants.kArminButton);

        //armup.onTrue(new InstantCommand(PIDArmLifterSubsystem::setSetpointVertical,PIDArmLifterSubsystem));
        //armdown.whileTrue(new InstantCommand(PIDArmLifterSubsystem::setSetpointGround,PIDArmLifterSubsystem));

        // double kStabilizationP = 0.025;
        // double kStabilizationI = 0.0001;
        // double kStabilizationD = 0.00;
        // armup
        // .whileTrue(
        //     new PIDCommand(
        //         new PIDController(
        //             kStabilizationP,
        //             kStabilizationI,
        //             kStabilizationD),
        //         // Close the loop on the turn rate
        //         ArmLifterSubsystem::getEncoderValue,
        //         // Setpoint is 0
        //         0,
        //         // Pipe the output to the turning controls
        //         output -> ArmLifterSubsystem.LiftArmUp(output),
        //         // Require the robot drive
        //         ArmLifterSubsystem));
        
        // armdown
        // .whileTrue(
        //     new PIDCommand(
        //         new PIDController(
        //             kStabilizationP,
        //             kStabilizationI,
        //             kStabilizationD),
        //         // Close the loop on the turn rate
        //         ArmLifterSubsystem::getEncoderValue,
        //         // Setpoint is 0
        //         60,
        //         // Pipe the output to the turning controls
        //         output -> ArmLifterSubsystem.LiftArmDown(output),
        //         // Require the robot drive
        //         ArmLifterSubsystem));
        //////////////////////////////////

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

        

        
        //armout.whileTrue(new ArmOutCMD(ArmExtensionSubsystem));
        //armin.whileTrue(new ArmInCMD(ArmExtensionSubsystem));
        
        
        //armup.whileTrue(new StartEndCommand(ArmLifterSubsystem::LiftArmUp, ArmLifterSubsystem::StopLiftArm, ArmLifterSubsystem));
        //armdown.whileTrue(new StartEndCommand(ArmLifterSubsystem::LiftArmDown, ArmLifterSubsystem::StopLiftArm, ArmLifterSubsystem));
        //armin.whileTrue(new StartEndCommand(ArmExtensionSubsystem::ExtArmIn, ArmExtensionSubsystem::ExtArmStop, ArmExtensionSubsystem));
        //armout.whileTrue(new StartEndCommand(ArmExtensionSubsystem::ExtArmOut, ArmExtensionSubsystem::ExtArmStop, ArmExtensionSubsystem));
        int kArmOutHighestPoleButton = 4;//this is the Y button, the triangle button or the Top button of the xbox controller
        int kArmOutMidestPoleButton = 2;//this is the B button, the Square button or the Right button of the xbox controller
        int kArmOutLowestPoleButton = 3; // X button, Left button of the xbox controller
        int kArmin = 1;// A button, bottom of 4 buttons
        JoystickButton armHighestout = new JoystickButton(driveController, kArmOutHighestPoleButton);
        JoystickButton armMidestOut = new JoystickButton(driveController, kArmOutMidestPoleButton);
        JoystickButton armLowestOut = new JoystickButton(driveController, kArmOutLowestPoleButton);
        JoystickButton armIn = new JoystickButton(driveController, kArmin);
        armHighestout.onTrue(new InstantCommand(PIDArmExtensionSubsystem::setSetpointHighestScore,PIDArmExtensionSubsystem));
        armMidestOut.onTrue(new InstantCommand(PIDArmExtensionSubsystem::setSetpointMidScore,PIDArmExtensionSubsystem));
        armLowestOut.onTrue(new InstantCommand(PIDArmExtensionSubsystem::setSetpointLowScore,PIDArmExtensionSubsystem));
        armIn.onTrue(new InstantCommand(PIDArmExtensionSubsystem::setSetpointIn,PIDArmExtensionSubsystem));
        
        //new JoystickButton(joystick1, 10).whileTrue(new InstantCommand(PIDArmExtensionSubsystem::enable,PIDArmExtensionSubsystem));
        int kHomeLifterPOVDirection = 0;//0 is up, 180 is down
        int kHomeExtensionPOVDirection = 180;//0 is up, 180 is down
        int kHomeLassoPOVDirection = 90;//0 is up, 180 is down
        
        POVButton HomeLifterPOV = new POVButton(driveController, kHomeLifterPOVDirection);
        POVButton HomeExtensionPOV = new POVButton(driveController, kHomeExtensionPOVDirection);
        POVButton HomeLassoPOV = new POVButton(driveController, kHomeLassoPOVDirection);
        
        HomeLifterPOV.whileTrue(new StartEndCommand(PIDArmLifterSubsystem::slowWindInBeyondSoftLimit, PIDArmLifterSubsystem::resetEncoder,PIDArmLifterSubsystem));
        HomeExtensionPOV.whileTrue(new StartEndCommand(PIDArmExtensionSubsystem::slowWindInBeyondSoftLimit, PIDArmExtensionSubsystem::resetEncoder,PIDArmExtensionSubsystem));
        HomeLassoPOV.whileTrue(new StartEndCommand(PIDLassoSubsystem::slowWindInBeyondSoftLimit, PIDLassoSubsystem::resetEncoder,PIDLassoSubsystem));

        JoystickButton turbobutton = new JoystickButton(driveController, 8);
        turbobutton.onTrue(new InstantCommand(driveSubsystem::changeturbomode,driveSubsystem));

        //new JoystickButton(joystick1, Constants.OperatorConstants.kresetLassoEncoderButton).whileTrue(new StartEndCommand(LassoSubsystem::slowWindInBeyondSoftLimit, LassoSubsystem::resetEncoder,LassoSubsystem));
        //new JoystickButton(joystick1, Constants.OperatorConstants.kresetLassoEncoderButton).whileTrue(new RunCommand(LassoSubsystem::resetEncoder,LassoSubsystem));
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