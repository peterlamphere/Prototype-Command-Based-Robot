package org.usfirst.frc.team6873.robot.subsystems;
import org.usfirst.frc.team6873.robot.Robot;
import org.usfirst.frc.team6873.robot.commands.DriveWithJoystick;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice; 

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.drive.*;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;  
	

/**
 * */
public class DriveSystem extends Subsystem {
	class PID  {
		static final double kP = 0.5;
		static final double kI = 0.0;
		static final double kD = 0.0;
	}
	final double pulsesPerRotation = 360*4 ; // We are using the US Digital E4T-360-250 to calculate PPR

	static final double circumferenceInInches = 6*Math.PI;
	static final double defaultSpeed = 0.55;
	static final double feetPerSecond = 3;
	public double getfeetPerSecond() { return feetPerSecond; }

	static final double degreesPerSecond = 230;
	public final double getdegreesPerSecond() { return degreesPerSecond; }
	
	WPI_TalonSRX  frontLeftMotor = new WPI_TalonSRX (2),  rearLeftMotor = new WPI_TalonSRX(1), frontRightMotor = new WPI_TalonSRX(3),  rearRightMotor = new WPI_TalonSRX(4);
	WPI_TalonSRX encoderTalonLeft;
	WPI_TalonSRX encoderTalonRight;
	
	
	SpeedControllerGroup leftMotors = new SpeedControllerGroup (frontLeftMotor, rearLeftMotor);
	SpeedControllerGroup rightMotors = new SpeedControllerGroup (frontRightMotor, rearRightMotor);
	
	PigeonIMU pigeon;
    PigeonIMU.FusionStatus fusionStatus = new PigeonIMU.FusionStatus(); 

    PigeonIMU.GeneralStatus genStatus = new PigeonIMU.GeneralStatus();


	DifferentialDrive myRobot = new DifferentialDrive ( leftMotors,rightMotors);
	
    // Put methods forthod controlling this subsystem
    // here. Call these from Commands.

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    	setDefaultCommand (new DriveWithJoystick());
    }
    public void initEncoder () {
    	encoderTalonLeft = frontLeftMotor;
    	encoderTalonRight = frontRightMotor;
    	rearLeftMotor.set(ControlMode.Follower, 2);// This will make the rear left motor move exactly like the front left motor
    	rearRightMotor.set(ControlMode.Follower, 3);// This will make the rear right motor move exactly like like the front right motor
    	encoderTalonLeft.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder,0,0);
    	encoderTalonLeft.config_kP(0, PID.kP,0);
    	encoderTalonLeft.config_kI(0, PID.kI,0);
    	encoderTalonLeft.config_kD(0, PID.kD,0); 
    	
    	encoderTalonRight.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder,0,0);
    	encoderTalonRight.config_kP(0, PID.kP,0);
    	encoderTalonRight.config_kI(0, PID.kI,0);
    	encoderTalonRight.config_kD(0, PID.kD,0); 
    }
    public void forward(double distance) {
    	int currentPositionLeft = encoderTalonLeft.getSelectedSensorPosition(0);
    	int currentPositionRight = encoderTalonRight.getSelectedSensorPosition(0);

    	//encoderTalon.getSelectedSensorPosition(0); getSensorCollection().getQuadraturePosition();
    	int targetPulseCount = (int) (distance * circumferenceInInches * pulsesPerRotation);
    	SmartDashboard.putNumber("Current Position Left", currentPositionLeft);
      	SmartDashboard.putNumber("Current Position Right", currentPositionRight);
      	 SmartDashboard.putNumber("Target position ", targetPulseCount);
    	encoderTalonLeft.set(ControlMode.Position, targetPulseCount);
     	encoderTalonRight.set(ControlMode.Position, targetPulseCount);
    }
    public void initGyro() {
		pigeon = new PigeonIMU (rearRightMotor);
    	if (pigeon != null) {
    		pigeon.setFusedHeading(0.0,10); 
    	

		    int loops = 0;
		    do  {
			    pigeon.getGeneralStatus(genStatus);	    
			    SmartDashboard.putString("Starting Pigeon State", genStatus.state.name());
			    Timer.delay(0.01);
		    } while (genStatus.state != PigeonIMU.PigeonState.Ready || loops++ < 100);
    	}
	    SmartDashboard.putString("Finished Pigeon Init", genStatus.state.name());

    }
/*    
    public boolean hasDrivenFarEnough(double distance) {
    	int currentPosition = encoderTalon.getSensorCollection().getQuadraturePosition();
    	int targetPulseCount = (int) (distance * circumferenceInInches * pulsesPerRotation);
    	SmartDashboard.putNumber("Current Position ", currentPosition);
    	SmartDashboard.putNumber("Target position ", targetPulseCount);
    	if (currentPosition >= targetPulseCount) {
    		return true;
    	}
    	return true;
    }
*/
    	
    public void tankDrive (double leftSpeed, double rightSpeed) {
		SmartDashboard.putNumber("Left Motors", leftSpeed);
 		SmartDashboard.putNumber("Right Motors", rightSpeed);
    	myRobot.tankDrive(leftSpeed,rightSpeed);
    	
    }
    public void arcadeDrive (double speed, double angle) {
		SmartDashboard.putNumber("Drive Speed", speed);
 		SmartDashboard.putNumber("Drive Angle", angle);
    	myRobot.arcadeDrive(speed,angle);
    	
    }
    
    public void forward () {
		SmartDashboard.putString("Drive System", "Going Forwards");
    	myRobot.arcadeDrive(defaultSpeed, 0);
    	
    }

    public void forwardwithGyro () {
    	final double KP = 0.03; // Constant for how fast to driving angle



		SmartDashboard.putString("Drive System", "Gyro Going Forwards");
	    pigeon.getFusedHeading(fusionStatus);
	    pigeon.getGeneralStatus(genStatus);
		
	    SmartDashboard.putString("Pigeon Error",  genStatus.lastError.name());
	    SmartDashboard.putString("Pigeon Booting",  genStatus.bCalIsBooting ? "True" : "False");
	    SmartDashboard.putString("Pigeon State", genStatus.state.name());
		
	    double currentAngle = fusionStatus.heading; 
	
		SmartDashboard.putString("Pigeon State", pigeon.getState().name());
				
		boolean angleIsGood = (pigeon.getState() == PigeonIMU.PigeonState.Ready);
		
    	if (angleIsGood) {
    			arcadeDrive(defaultSpeed, currentAngle*-KP);
    	}  else {
				arcadeDrive(defaultSpeed, 0);
    	}
    	
		SmartDashboard.putNumber("Angle", currentAngle);
		SmartDashboard.putString("AngleIsGood", (angleIsGood ? "Yes" :"No") );
        	
    	
    	
    }
    public void backward() {
		SmartDashboard.putString("Drive System", "Going Backwards");
    	myRobot.arcadeDrive(-defaultSpeed, 0);
    }
    public void turnLeft() {
		SmartDashboard.putString("Drive System", "Turning Left");
    	myRobot.arcadeDrive(defaultSpeed, -1.0);
    }
    public void turnRight() {
		SmartDashboard.putString("Drive System", "Turning Right");
    	myRobot.arcadeDrive(defaultSpeed, 1.0);
    }
    
    public void stop() {
		SmartDashboard.putString("Drive System", "Stopping");
    	myRobot.tankDrive(0, 0);
    }
    
}

