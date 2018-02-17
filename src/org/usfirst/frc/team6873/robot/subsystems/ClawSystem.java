package org.usfirst.frc.team6873.robot.subsystems;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class ClawSystem extends Subsystem {
	Spark topController = new Spark (1);
	Spark bottomController = new Spark (2);
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    public void forward() {
		SmartDashboard.putString(" Claw System", "Going Forwards");
    	topController.set(1.0);
    	bottomController.set(1.0);
    }
    public void backward() {
    	topController.set(-1.0); 		SmartDashboard.putString(" Claw System", "Going backward");
    	bottomController.set(-1.0);
    	
    }
    public void stop() {
    	topController.set(0.0); 		SmartDashboard.putString(" Claw System", "Stopping");
    	bottomController.set(0.0);
    	
    }



    }


