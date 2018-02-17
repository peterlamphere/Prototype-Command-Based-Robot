package org.usfirst.frc.team6873.robot.commands;

import org.usfirst.frc.team6873.robot.Robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class DriveWithJoystick extends Command {
	Command elevatorUpCommand = new ElevatorUp();
    Command elevatorDownCommand = new ElevatorDown();
    Command WinchUpCommand = new WinchUp();
    Command WinchDownCommand = new WinchDown();

    public DriveWithJoystick() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires (Robot.driveSubsystem);
    	    	
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	
    	if (Robot.oi.leftStick.getTriggerPressed() || Robot.oi.rightStick.getTriggerPressed()) {
    		//drive straight
    		double averageSpeed = (Robot.oi.leftStick.getY() + Robot.oi.rightStick.getY())/2;
    		Robot.driveSubsystem.forwardwithGyro(averageSpeed);
    	} else {
    		Robot.driveSubsystem.tankDrive(-Robot.oi.leftStick.getY(),-Robot.oi.rightStick.getY());
    	}
    	
    	int LeftPOV = Robot.oi.leftStick.getPOV();
    	if (LeftPOV >= 0) {
    		if (LeftPOV == 0) {
    			//ElevatorUp
    			elevatorUpCommand.start();
    		} else if (LeftPOV == 180) {
    			//ElevatorDown
    			elevatorDownCommand.start();
    		}
    	} else {
    		elevatorDownCommand.cancel();
    		elevatorUpCommand.cancel();
    		
    	}
    	
    	
    	int RightPOV = Robot.oi.rightStick.getPOV();
    	if (RightPOV >= 0) {
    		if (RightPOV == 0) {
    	    		//WinchUp
    	    		WinchUpCommand.start();
    	    	} else if (RightPOV == 180) {
    	    		//WinchDown
    	    		WinchDownCommand.start();
    	    	}
    	} else {
    		//WinchStop    
    		WinchDownCommand.cancel();
    		WinchUpCommand.cancel(); 
    	}
    	
    	
    	double winchValue = Robot.oi.gamepad.getY(GenericHID.Hand.kLeft);
    	if (winchValue > 0.05 ) {
    		
    	}
    	
    	
    	
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
