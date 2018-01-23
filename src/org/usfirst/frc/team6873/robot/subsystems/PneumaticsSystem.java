package org.usfirst.frc.team6873.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Timer;


/**
 *
 */
public class PneumaticsSystem extends Subsystem {
	Compressor comp = new Compressor(0);
	DoubleSolenoid claw = new DoubleSolenoid(0,1);
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    public void openClaw() {
    	SmartDashboard.putString("Pneumatics System", "Open Claw");
    	claw.set(DoubleSolenoid.Value.kForward);
    	Timer.delay(0.1);

    }	
    public void closeClaw() {
    	SmartDashboard.putString("Pneumatics System", "Close Claw");
    	claw.set(DoubleSolenoid.Value.kReverse);
    	Timer.delay(0.1);
    
    }
    public void turn_offClaw() {
    	SmartDashboard.putString("Pneutics System", "Turn off Claw");
    	claw.set(DoubleSolenoid.Value.kOff);
    	Timer.delay(0.1);

    }	
}

