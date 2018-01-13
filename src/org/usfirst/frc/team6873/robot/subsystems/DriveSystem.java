package org.usfirst.frc.team6873.robot.subsystems;

import com.ctre.CANTalon;


import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;  
	

/**
 *
 */
public class DriveSystem extends Subsystem {
	static double defaultSpeed = 0.5;
	CANTalon  frontLeftMotor = new CANTalon (2),  rearLeftMotor = new CANTalon(1), frontRightMotor = new CANTalon(3),  rearRightMotor = new CANTalon(4);
	
	RobotDrive myRobot = new RobotDrive( frontLeftMotor,  rearLeftMotor,  frontRightMotor,  rearRightMotor );

    // Put methods forthod controlling this subsystem
    // here. Call these from Commands.

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    public void forward () {
		SmartDashboard.putString("Drive System", "Going Forwards");
    	myRobot.drive(defaultSpeed, 0);
    	
    }
    
    public void backward() {
		SmartDashboard.putString("Drive System", "Going Backwards");
    	myRobot.drive(-defaultSpeed, 0);
    }
    public void turnLeft() {
		SmartDashboard.putString("Drive System", "Turning Left");
    	myRobot.drive(defaultSpeed, -1.0);
    }
    public void turnRight() {
		SmartDashboard.putString("Drive System", "Turning Right");
    	myRobot.drive(defaultSpeed, 1.0);
    }
    
    public void stop() {
		SmartDashboard.putString("Drive System", "Stopping");
    	myRobot.drive(0, 0);
    }
    
}

