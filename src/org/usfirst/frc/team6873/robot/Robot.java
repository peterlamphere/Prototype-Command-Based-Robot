
package org.usfirst.frc.team6873.robot;

import edu.wpi.first.wpilibj.IterativeRobot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team6873.robot.commands.*;

import org.usfirst.frc.team6873.robot.subsystems.*;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {

	public static final DriveSystem driveSubsystem = new DriveSystem();
	//public static final SmallMotorSystem smallMotorSubsystem = new SmallMotorSystem();
	public static final PneumaticsSystem pnuematicsSubsystem = new PneumaticsSystem();
	public static final ElevatorSystem elevatorSubsystem = new ElevatorSystem();
	public static final ClawSystem clawSubsystem = new ClawSystem();
	public static final WinchSystem winchSubsystem = new WinchSystem();
	public static OI oi;
	public static Joystick stick = new Joystick(0);
	
	Command autonomousCommand;
	SendableChooser<Command> chooser = new SendableChooser<>();

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	
	void Disabled() { while(isDisabled());}

	@Override
	public void robotInit() {
		oi = new OI();
		
		chooser.addDefault("Forward 10 foot", new DriveForward(10));
		chooser.addObject("Forward To Object in 6 Inches", new DriveForwardUntilObject(6.0));
		chooser.addObject("Backward 10 foot", new DriveBackward(10));
		chooser.addObject("Forward 10 foot with Gyro", new DriveForwardWithGyro(10));
		chooser.addObject("Forward 10 foot with Encoder", new DriveForwardWithEncoder(10));

		chooser.addObject("Turn Left 90 degrees", new TurnLeft(90));
		chooser.addObject("Turn Right 90 degrees", new TurnRight(90));
		//chooser.addObject("Test Claw", new TestClaw());
		//chooser.addObject("Test SmallMotors", new TestSmallMotor());
		chooser.addObject("Autonomous position 1 (Right)", new Autonomous1Right());
		// chooser.addObject("My Auto", new MyAutoCommand());
		SmartDashboard.putData("Auto mode", chooser);
		chooser.addObject("Autonomous position 1 (Left)", new Autonomous1Left());
		// chooser.addObject("My Auto", new MyAutoCommand());
		SmartDashboard.putData("Auto mode", chooser);
	
	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	@Override
	public void disabledInit() {

	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString code to get the auto name from the text box below the Gyro
	 *
	 * You can add additional auto modes by adding additional commands to the
	 * chooser code above (like the commented example) or additional comparisons
	 * to the switch structure below with additional strings & commands.
	 */
	@Override
	public void autonomousInit() {
		autonomousCommand = chooser.getSelected();

		/*
		 * String autoSelected = SmartDashboard.getString("Auto Selector",
		 * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
		 * = new MyAutoCommand(); break; case "Default Auto": default:
		 * autonomousCommand = new ExampleCommand(); break; }
		 */

		// schedule the autonomous command (example)
		if (autonomousCommand != null)
			autonomousCommand.start();
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
	}

	@Override
	public void teleopInit() {
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if (autonomousCommand != null)
			autonomousCommand.cancel();
		
		
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
		LiveWindow.run();
	}
}
