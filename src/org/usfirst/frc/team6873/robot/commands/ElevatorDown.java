package org.usfirst.frc.team6873.robot.commands;

import org.usfirst.frc.team6873.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class ElevatorDown extends Command {

    public ElevatorDown() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	SmartDashboard.putString("Command", "Starting ElevatorDown Command");
    	requires(Robot.elevatorSubsystem);
   
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	SmartDashboard.putString("Command", "Starting Elevator Down Command");
    	Robot.elevatorSubsystem.initDefaultCommand();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
     	SmartDashboard.putString("Command", "Executing ElevatorDown Command");
    Robot.elevatorSubsystem.backward();
        
}

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	SmartDashboard.putString("Command", "Elevator System Stoping ");

    	Robot.elevatorSubsystem.stop();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
    
}
