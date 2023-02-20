package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.ArmExtensionSubsystem;



public class ArmExtensionJoystickCmd extends CommandBase {

    private final ArmExtensionSubsystem ArmExtensionSubsystem;
    private final Supplier<Double> speedFunction;

    public ArmExtensionJoystickCmd(ArmExtensionSubsystem ArmExtensionSubsystem, 
             Supplier<Double> speedFunction) {
        this.speedFunction = speedFunction;
        this.ArmExtensionSubsystem = ArmExtensionSubsystem;
        addRequirements(ArmExtensionSubsystem);
    }

    @Override
    public void initialize() {
        //System.out.println("LassoJoystickCmd started!");
    }

    @Override
    public void execute() {
        ArmExtensionSubsystem.SetSpeed(speedFunction.get());
    }

    @Override
    public void end(boolean interrupted) {
        //System.out.println("LassoJoystickCmd ended!");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
