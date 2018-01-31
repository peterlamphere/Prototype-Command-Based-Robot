package org.usfirst.frc.team6873.robot.commands;

import org.usfirst.frc.team6873.robot.Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class DriveForwardWithEncoder extends Command {
	double feet;
    public DriveForwardWithEncoder(double _feet) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.driveSubsystem);
    	feet = _feet;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	
    	Robot.driveSubsystem.initEncoder();
		SmartDashboard.putString("Command", "Starting Encoder Drive Forward Command");
		SmartDashboard.putNumber("Feet", feet);
		Robot.driveSubsystem.forward(feet);  // Call encoder enabled method from Subsystem
		Timer.delay(2);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
		SmartDashboard.putString("Command", "Running Encoder Drive Forward Command");
		//Robot.driveSubsystem.forward();
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	return false;
    	//return Robot.driveSubsystem.hasDrivenFarEnough(feet*12);  // Alternate method for using Talon-mounted encoders
    }

    // Called once after isFinished returns true
    protected void end() {
		SmartDashboard.putString("Command", "Stopping Encoder Drive Forward Command");

    	Robot.driveSubsystem.stop();
    	
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {   
		SmartDashboard.putString("Command", "Interrupting Encoder Drive Forward Command");
		Robot.driveSubsystem.stop();
    }
}
