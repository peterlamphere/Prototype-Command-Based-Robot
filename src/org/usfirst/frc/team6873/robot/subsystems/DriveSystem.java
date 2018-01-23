package org.usfirst.frc.team6873.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;


import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;  
	

/**
 * */
public class DriveSystem extends Subsystem {
	static double defaultSpeed = 0.55;
	static double feetPerSecond = 7;
	public double getfeetPerSecond() { return feetPerSecond; }

	static double degreesPerSecond = 230;
	public double getdegreesPerSecond() { return degreesPerSecond; }
	
	WPI_TalonSRX  frontLeftMotor = new WPI_TalonSRX (2),  rearLeftMotor = new WPI_TalonSRX(1), frontRightMotor = new WPI_TalonSRX(3),  rearRightMotor = new WPI_TalonSRX(4);
	
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

