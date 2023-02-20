package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.ArmLifterSubsystem;



public class ArmLifterJoystickCmd extends CommandBase {

    private final ArmLifterSubsystem ArmLifterSubsystem;
    private final Supplier<Double> speedFunction;

    public ArmLifterJoystickCmd(ArmLifterSubsystem ArmLifterSubsystem, 
             Supplier<Double> speedFunction) {
        this.speedFunction = speedFunction;
        this.ArmLifterSubsystem = ArmLifterSubsystem;
        addRequirements(ArmLifterSubsystem);
    }
    @Override
    public void initialize() {
        //System.out.println("LassoJoystickCmd started!");
    }

    @Override
    public void execute() {
        //ArmLifterSubsystem.SetSpeed(speedFunction.get());
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
