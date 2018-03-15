package org.usfirst.frc.team6873.robot.commands;

import org.usfirst.frc.team6873.robot.Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team6873.robot.subsystems.ElevatorSystem;

/**
 *
 */
public class TestSmallMotor extends Command {
	Timer timer = new Timer();

    public TestSmallMotor() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.elevatorSubsystem);

    }

    // Called just before this Command runs the first time
    protected void initialize() {
		SmartDashboard.putString("Small Motor", "Starting Small Motor Test Command");

    	timer.reset();
    	timer.start();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
		SmartDashboard.putNumber("Timer", timer.get());	
		
		Robot.elevatorSubsystem.backward();
/*
    	if (((int) timer.get() % 6) < 3) {
    		SmartDashboard.putString("Small Motor", "Running Forward Small Motor Test Command");
        	Robot.smallMotorSubsystem.forward();

    	} else {
    		SmartDashboard.putString("Small Motor", "Running Backwards Small Motor Test Command");
        	Robot.smallMotorSubsystem.backward();
    	}	

*/
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return timer.get() < 0.1;
    }

    // Called once after isFinished returns true
    protected void end() {
       		timer.stop();
    		SmartDashboard.putString("Small Motor", "Stopping Drive Forward Command");
 //       	Robot.smallMotorSubsystem.stop();

    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	
    }
    
}
