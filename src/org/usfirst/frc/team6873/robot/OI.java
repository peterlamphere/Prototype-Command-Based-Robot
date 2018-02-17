package org.usfirst.frc.team6873.robot;

import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;


import org.usfirst.frc.team6873.robot.commands.*;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
	//// CREATING BUTTONS
	// One type of button is a joystick button which is any button on a
	//// joystick.
	// You create one by telling it which joystick it's on and which button
	// number it is.
	public Joystick leftStick = new Joystick(0), rightStick = new Joystick(1);
	public XboxController gamepad = new XboxController (2);
	public Button buttonOpenClaw = new JoystickButton(rightStick, 3);
	public Button buttonCloseClaw = new JoystickButton(rightStick, 2);
    public Button buttonClawEnter = new JoystickButton(leftStick, 3);
    public Button buttonClawExit = new JoystickButton(leftStick, 2);
    public Button buttonTurnRight45 = new JoystickButton(rightStick, 6);
    public Button buttonTurnRight90= new JoystickButton(rightStick, 4);
    public Button buttonTurnLeft45 = new JoystickButton(leftStick, 6);
    public Button buttonTurnLeft90= new JoystickButton(leftStick, 4);
    
    
    
    
    // There are a few additional built in buttons you can use. Additionally,
	// by subclassing Button you can create custom triggers and bind those to
	// commands the same as any other Button.

	//// TRIGGERING COMMANDS WITH BUTTONS
	// Once you have a button, it's trivial to bind it to a button in one of
	// three ways:

	// Start the command when the button is pressed and let it run the command
	// until it is finished as determined by it's isFinished method.
    public OI() {
		buttonOpenClaw.whenPressed(new OpenClaw());
	    buttonCloseClaw.whenPressed(new CloseClaw());
	    buttonClawEnter.whenPressed(new ClawEnter());
	    buttonClawExit.whenPressed(new ClawExit());
	    buttonTurnRight45.whenPressed (new TurnRight(45));
	    buttonTurnRight90.whenPressed (new TurnRight(90));
	    buttonTurnLeft45.whenPressed (new TurnLeft(45));
	    buttonTurnLeft90.whenPressed (new TurnLeft(90));
	    
	    
	    
    }
	// Run the command while the button is being held down and interrupt it once
	// the button is released.
	// button.whileHeld(new ExampleCommand());

	// Start the command when the button is released and let it run the command
	// until it is finished as determined by it's isFinished method.
	// button.whenReleased(new ExampleCommand());
}
