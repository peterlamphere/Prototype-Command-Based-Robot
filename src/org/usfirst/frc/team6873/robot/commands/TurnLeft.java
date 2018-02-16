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
	double targetAngle = 0.0;
	double tolerance = 0.5; // Half a degree tolerance
	
    public TurnLeft(double _degrees) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);\
    	requires(Robot.driveSubsystem);
    	targetAngle = _degrees;
    	
    }

    // Called just before this Command runs the first time
    protected void initialize() {
		SmartDashboard.putString("Command", "Starting Drive TurnLeft Command");
    	Robot.driveSubsystem.initGyro();

    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
		SmartDashboard.putString("Command", "Running Drive TurnLeft Command");
		SmartDashboard.putNumber("Target Angle", targetAngle);
		double percentOfTurn = Robot.driveSubsystem.getHeadingAngle() / targetAngle;
		SmartDashboard.putNumber("Percent of Turn", percentOfTurn);

		if (percentOfTurn > 0.60) {
			if (percentOfTurn < 1.0) {
				double power = 1.00 / (0.60-1.0) * (percentOfTurn - 1.00);
				Robot.driveSubsystem.turnLeft(power);
			} else Robot.driveSubsystem.stop();
		} else Robot.driveSubsystem.turnLeft();
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	return (Robot.driveSubsystem.getHeadingAngle() > targetAngle);
    }

    // Called once after isFinished returns true
    protected void end() {
    	timer.stop();
		SmartDashboard.putString("Command", "Stopping Drive TurnLeft Command");

    	Robot.driveSubsystem.stop();
    }
}
