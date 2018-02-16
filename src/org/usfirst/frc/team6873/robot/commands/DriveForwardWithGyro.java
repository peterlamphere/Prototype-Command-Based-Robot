package org.usfirst.frc.team6873.robot.commands;
import com.ctre.phoenix.sensors.*;
import org.usfirst.frc.team6873.robot.Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class DriveForwardWithGyro extends Command {

	Timer timer = new Timer();
	double time = 0;
	double rampUpTime = 0.5;
	
    
    public DriveForwardWithGyro(double _feet) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);\
    	requires(Robot.driveSubsystem);
    	time = _feet/Robot.driveSubsystem.getfeetPerSecond();
    
    }

    // Called just before this Command runs the first time
    protected void initialize() {
		SmartDashboard.putString("Command", "Starting Drive Forward Command");
		SmartDashboard.putNumber("Time Limit", time);
    	timer.reset();
    	timer.start();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
		SmartDashboard.putString("Command", "Running Gyro Drive Forward Command");
		SmartDashboard.putNumber("Timer", timer.get());
		if (timer.get() < rampUpTime)
			Robot.driveSubsystem.forwardPartialPower(timer.get()/rampUpTime);
		else
			Robot.driveSubsystem.forwardwithGyro();
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	return (timer.get() > time);
    }

    // Called once after isFinished returns true
    protected void end() {
    	timer.stop();
		SmartDashboard.putString("Command", "Stopping Drive Forward Command");

    	Robot.driveSubsystem.stop();
    	
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {   
		SmartDashboard.putString("Command", "Interrupting Drive Forward Command");
		timer.stop();
		Robot.driveSubsystem.stop();
    }
}
