package org.usfirst.frc.team6873.robot.commands;

import org.usfirst.frc.team6873.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class WinchUp extends Command {

    public WinchUp() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	SmartDashboard.putString("Command", "Starting WinchUp Command");
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.winchSubsystem.initDefaultCommand();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
     	SmartDashboard.putString("Command", "Executing WinchUp Command");
     	Robot.winchSubsystem.goUp();
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	SmartDashboard.putString("Ending", "Ending Winch Up Command");
    	Robot.winchSubsystem.stop();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
