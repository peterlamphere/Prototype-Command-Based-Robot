package org.usfirst.frc.team6873.robot.commands;

import org.usfirst.frc.team6873.robot.Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class TurnRight extends Command {
	Timer timer = new Timer();
	double targetAngle = 0.0;
	double time = 0.5;	
	double tolerance = 0.5; // Half a degree tolerance

	
    public TurnRight(double _degrees) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);\
    	requires(Robot.driveSubsystem);
    	//time = _degrees/Robot.driveSubsystem.getdegreesPerSecond() ;
    	targetAngle =-_degrees;

    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.driveSubsystem.initGyro();
		SmartDashboard.putString("Command", "Starting Drive TurnRight Command");
		//SmartDashboard.putNumber("Time Limit", time);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
		SmartDashboard.putString("Command", "Running Drive TurnRight Command");
		SmartDashboard.putNumber("Target Angle", targetAngle);
		double percentOfTurn = Math.abs(Robot.driveSubsystem.getHeadingAngle() / targetAngle);
		SmartDashboard.putNumber("Percent of Turn", percentOfTurn);

		
		Robot.driveSubsystem.turnRight();
		if (percentOfTurn > 0.75) {
			Robot.driveSubsystem.turnRight(Robot.driveSubsystem.slowSpeed);
		} else if (percentOfTurn > 1.0) {
			Robot.driveSubsystem.turnLeft(Robot.driveSubsystem.slowSpeed);
		} else {	
			Robot.driveSubsystem.turnRight(Robot.driveSubsystem.fastSpeed);
		}
    	//timer.reset();
    	//timer.start();
    }

    // Called repeatedly when this Command is scheduled to run
    //protected void execute() {
		///SmartDashboard.putString("Command", "Running Drive TurnRight Command");
		//SmartDashboard.putNumber("Timer", timer.get());
    	//Robot.driveSubsystem.turnRight();
    

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	return ( Math.abs(Robot.driveSubsystem.getHeadingAngle() - targetAngle) < tolerance );
    }

    // Called once after isFinished returns true
    protected void end() {
    	timer.stop();
		SmartDashboard.putString("Command", "Stopping Drive TurnRight Command");

    	Robot.driveSubsystem.stop();
  
    }
}
