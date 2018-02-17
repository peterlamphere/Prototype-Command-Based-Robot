package org.usfirst.frc.team6873.robot.commands;

import org.usfirst.frc.team6873.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class ClawExit extends Command {

    public ClawExit() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	SmartDashboard.putString("Command", "Starting ClawExit Command");
    	Robot.clawSubsystem.initDefaultCommand();
    }

    // Called just before this Command runs the first time
    protected void initialize() {
     	SmartDashboard.putString("Command", "Executing ClawExit Command");
     	Robot.clawSubsystem.backward();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	SmartDashboard.putString("Close", "Ending Exit Claw Command");
    	Robot.clawSubsystem.stop();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
