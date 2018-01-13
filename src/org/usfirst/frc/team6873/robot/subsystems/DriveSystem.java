package org.usfirst.frc.team6873.robot.subsystems;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.command.Subsystem;  
	

/**
 *
 */
public class DriveSystem extends Subsystem {
	CANTalon  frontLeftMotor = new CANTalon (2),  rearLeftMotor = new CANTalon(1), frontRightMotor = new CANTalon(3),  rearRightMotor = new CANTalon(4);
	
	RobotDrive myRobot = new RobotDrive( frontLeftMotor,  rearLeftMotor,  frontRightMotor,  rearRightMotor );

    // Put methods forthod controlling this subsystem
    // here. Call these from Commands.

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
}

