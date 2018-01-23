package org.usfirst.frc.team6873.robot.commands;

import org.usfirst.frc.team6873.robot.Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class TurnLeft extends Command {
	Timer timer = new Timer();
	double time = 0;
	
    public TurnLeft(double _degrees) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);\
    	requires(Robot.driveSubsystem);
    	time = _degrees/Robot.driveSubsystem.getdegreesPerSecond() ;
    	
    }

    // Called just before this Command runs the first time
    protected void initialize() {
		SmartDashboard.putString("Command", "Starting Drive TurnLeft Command");
		SmartDashboard.putNumber("Time Limit", time);

    	timer.reset();
    	timer.start();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
		SmartDashboard.putString("Command", "Running Drive TurnLeft Command");
		SmartDashboard.putNumber("Timer", timer.get());
    	Robot.driveSubsystem.turnLeft();
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	return (timer.get() > time);
    }

    // Called once after isFinished returns true
    protected void end() {
    	timer.stop();
		SmartDashboard.putString("Command", "Stopping Drive TurnLeft Command");

    	Robot.driveSubsystem.stop();
    }
}
