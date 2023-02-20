package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.JoeColorSensor;
import frc.robot.Subsystems.LassoSubsystem;



public class LassoJoystickCmd extends CommandBase {

    private final LassoSubsystem lassoSubsystem;
    private final JoeColorSensor Colorsensor;
    private final Supplier<Double> speedFunction;

    public LassoJoystickCmd(LassoSubsystem lassoSubsystem, 
            JoeColorSensor Colorsensor, Supplier<Double> speedFunction) {
        this.Colorsensor = Colorsensor;
        this.speedFunction = speedFunction;
        this.lassoSubsystem = lassoSubsystem;
        addRequirements(lassoSubsystem);
    }

    @Override
    public void initialize() {
        //System.out.println("LassoJoystickCmd started!");
    }

    @Override
    public void execute() {
        lassoSubsystem.SetlassoSpeed(Colorsensor,speedFunction.get());
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
