package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

import edu.wpi.first.math.filter.SlewRateLimiter;

public class ArcadeDriveCmd extends CommandBase {

    private final DriveSubsystem driveSubsystem;
    private final Supplier<Double> speedFunction, turnFunction;
    SlewRateLimiter speedLimiter = new SlewRateLimiter(10);
    SlewRateLimiter turnLimiter = new SlewRateLimiter(10);
    public ArcadeDriveCmd(DriveSubsystem driveSubsystem, //
            Supplier<Double> speedFunction, Supplier<Double> turnFunction) {
        this.speedFunction = speedFunction;
        this.turnFunction = turnFunction;
        this.driveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        //System.out.println("ArcadeDriveCmd started!");
    }

    @Override
    public void execute() {
        double realTimeSpeed = speedLimiter.calculate(speedFunction.get());
        double realTimeTurn = turnLimiter.calculate(turnFunction.get());

        //double left = realTimeSpeed + realTimeTurn;
        //double right = realTimeSpeed - realTimeTurn;
        driveSubsystem.ArcadeDrivemotors(realTimeSpeed, realTimeTurn);
        //driveSubsystem.setMotors(left, right);
    }

    @Override
    public void end(boolean interrupted) {
        //System.out.println("ArcadeDriveCmd ended!");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
