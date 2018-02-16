package org.usfirst.frc.team6873.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Spark;

/**
 *
 */
public class SmallMotorSystem extends Subsystem {
	Spark controller = new Spark (0);
	
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    public void forward() {
		SmartDashboard.putString("Small System", "Going Forwards");
    	controller.set(0.5);
    	
    }
    public void backward() {
    	controller.set(-0.5); 		SmartDashboard.putString("Small System", "Going backward");
    	
    	
    }
    public void stop() {
    	controller.set(0.0); 		SmartDashboard.putString("Small System", "Stopping");
    	
    	
    }
}

