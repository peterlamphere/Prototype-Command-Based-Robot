package org.usfirst.frc.team6873.robot.subsystems;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class ElevatorSystem extends Subsystem {
		Spark controller = new Spark (0);
		
	    // Put methods for controlling this subsystem
	    // here. Call these from Commands.

	    public void initDefaultCommand() {
	        // Set the default command for a subsystem here.
	        //setDefaultCommand(new MySpecialCommand());
	    }
	    public void forward() {
			SmartDashboard.putString("Elevator System", "Going Up");
	    	controller.set(-0.25);
	    	
	    }
	    public void backward() {
	    	controller.set(0.25); 		SmartDashboard.putString("Elevator System", "Going down");
	    	
	    	
	    }
	    public void stop() {
	    	controller.set(0.0); 		SmartDashboard.putString("Elevator System", "Stopping");
	    	
	    	
	    }


}

