package org.usfirst.frc.team6873.robot.commands;

import org.usfirst.frc.team6873.robot.Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class DriveForwardUntilObject extends Command {
	
		Timer timer = new Timer();
		double time = 0;
		static double rampUpTime = 1.0;
		double inchestoTarget = 0;
		double tolerance = 0;
		final static double tolerancePercent = 5.0/100.0;
		final static double ultrasonicOffset = 19.0; // How far back are the sensors from the front of the robot?
		
	    public DriveForwardUntilObject (double _inches) {
	        // Use requires() here to declare subsystem dependencies
	        // eg. requires(chassis);\
	    	requires(Robot.driveSubsystem);
	    	 inchestoTarget = _inches + ultrasonicOffset;
	    	 tolerance = tolerancePercent*inchestoTarget;
	    	 Robot.driveSubsystem.initUltrasonic();
	    
	    
	    
	    }

	    double leftPower = 1.0;
	    double rightPower = 1.0;
	    
	    // Called just before this Command runs the first time
	    protected void initialize() {
			SmartDashboard.putString("Command", "Starting Drive Forward Until Object Command");
			SmartDashboard.putNumber("Time Limit", time);

	    	timer.reset();
	    	timer.start();
	    }

	    // Called repeatedly when this Command is scheduled to run
	    protected void execute() {
			SmartDashboard.putString("Command", "!!!Running Drive Forward Until Object Command");
			SmartDashboard.putNumber("Timer", timer.get());

			final double slowSpeed = 0.5, fastSpeed = 0.75;
			rightPower = leftPower = fastSpeed;
			
		
			
			double distanceLeft = Robot.driveSubsystem.distanceUntilObjectLeft();
			double distanceRight = Robot.driveSubsystem.distanceUntilObjectRight();
			
			if(distanceLeft > inchestoTarget * 2.5 && distanceRight > inchestoTarget * 2.5) {
				leftPower = rightPower = fastSpeed;
			} else {
				if(distanceLeft > inchestoTarget + tolerance) {
						leftPower= slowSpeed;
				} else if(distanceLeft < inchestoTarget - tolerance) {
							leftPower= -slowSpeed;
				} else {
							leftPower= 0.0;
				}
				
				if(distanceRight > inchestoTarget + tolerance) {
						rightPower =slowSpeed;
				} else if(distanceRight < inchestoTarget - tolerance) {
							rightPower= -slowSpeed;
				}	 else {
							rightPower= 0.0;
				}
			}
			SmartDashboard.putNumber("Right Power", rightPower);
			SmartDashboard.putNumber("Left Power", leftPower);
			
			Robot.driveSubsystem.tankDrive(leftPower,rightPower);

	    }

	    // Make this return true when this Command no longer needs to run execute()
	    protected boolean isFinished() {
	    	return (leftPower == 0.0 && rightPower == 0.0);
	    }

	    // Called once after isFinished returns true
	    protected void end() {
	    	timer.stop();
			SmartDashboard.putString("Command", "Stopping Drive Forward Until Object Command");

	    	Robot.driveSubsystem.stop();
	    	
	    }

	    // Called when another command which requires one or more of the same
	    // subsystems is scheduled to run
	    protected void interrupted() {   
			SmartDashboard.putString("Command", "Interrupting Drive Forward Until Object Command");
			timer.stop();
			Robot.driveSubsystem.stop();
	    }
	}