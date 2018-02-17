package org.usfirst.frc.team6873.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class WinchSystem extends Subsystem {
	WPI_TalonSRX  winchMotor = new WPI_TalonSRX (5);
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    public void goUp() {
		SmartDashboard.putString(" Winch System", "Going Up");
		winchMotor.set(0.6);
    	
    }
    public void goDown() {
		SmartDashboard.putString(" Winch System", "Going Down");
		winchMotor.set(-0.6);
    	
    }
    public void stop() {
		SmartDashboard.putString(" Winch System", "Going Down");
		winchMotor.set(0.0);
    	
    }

}

