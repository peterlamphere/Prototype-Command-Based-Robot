package org.usfirst.frc.team6873.robot.subsystems;
import org.usfirst.frc.team6873.robot.Robot;
import org.usfirst.frc.team6873.robot.commands.DriveWithJoystick;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice; 

import com.ctre.phoenix.sensors.PigeonIMU;



import edu.wpi.first.wpilibj.drive.*;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.Ultrasonic.Unit;
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
	public final double slowSpeed = 0.7; public final double fastSpeed = 0.85;

	final double pulsesPerRotation = 360; // We are using the US Digital E4T-360-250 to calculate PPR
	final int defaultTimeout = 10; // How long to wait before sensor method calls give up
	
	int startingPosition = 0;
    double startingAngle = 0;

	static final double circumferenceInInches = 6*Math.PI;
	static final double defaultSpeed = 0.8;
	static final double	defaultTurnSpeed = 0.6;
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

    Ultrasonic ultraLeft; //trigger channel 0, echo channel 1
    Ultrasonic ultraRight; //trigger channel 2, echo channel 3
    DifferentialDrive myRobot = new DifferentialDrive ( leftMotors,rightMotors);
	

	
    // Put methods forthod controlling this subsystem
    // here. Call these from Commands.
    public DriveSystem () {
    	//initGyro();
    	//initEncoder();
    	//initUltrasonic();
    }
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    	setDefaultCommand (new DriveWithJoystick());
    }
    
    public void initUltrasonic() {
    	if (ultraLeft == null) {

    		ultraLeft = new Ultrasonic (0,1);
    		ultraLeft.setEnabled(true); // turn on the sensor
    		//ultraLeft.setAutomaticMode(true);
    		ultraLeft.setDistanceUnits(Unit.kInches);
    	}
    	if (ultraRight == null) {
    		ultraRight = new Ultrasonic (2,3);
    		ultraRight.setEnabled(true); // turn on the sensor
    		ultraRight.setAutomaticMode(true);
    		ultraRight.setDistanceUnits(Unit.kInches);
    	}
    }
    public void initEncoder () {
    	
    	encoderTalonLeft = frontLeftMotor;
    	encoderTalonRight = frontRightMotor;

    	
    	rearLeftMotor.set(ControlMode.Follower, 3);// This will make the rear left motor move exactly like the front left motor
    	rearRightMotor.set(ControlMode.Follower, 3);// This will make the rear right motor move exactly like like the front right motor
    	encoderTalonLeft.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder,0,defaultTimeout);
//    	encoderTalonLeft.config_kP(0, PID.kP,defaultTimeout);
//    	encoderTalonLeft.config_kI(0, PID.kI,defaultTimeout);
//    	encoderTalonLeft.config_kD(0, PID.kD,defaultTimeout); 
    	encoderTalonLeft.setSelectedSensorPosition(0, 0, defaultTimeout);
   	
    	encoderTalonRight.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder,0, defaultTimeout);
//    	encoderTalonRight.config_kP(0, PID.kP,defaultTimeout);
//   	encoderTalonRight.config_kI(0, PID.kI,defaultTimeout);
//    	encoderTalonRight.config_kD(0, PID.kD,defaultTimeout); 
    	encoderTalonLeft.setSelectedSensorPosition(0, 0, defaultTimeout);

    	startingPosition = -encoderTalonLeft.getSensorCollection().getQuadraturePosition();

    }
    
    public boolean encoderEnabled() {	
    	return (encoderTalonRight != null); // to be implemented.
    }
    public void forward(double distance) {
    	int currentPositionLeft = encoderTalonLeft.getSelectedSensorPosition(0);
    	int currentPositionRight = encoderTalonRight.getSelectedSensorPosition(0);

    	//encoderTalon.getSelectedSensorPosition(0); getSensorCollection().getQuadraturePosition();
    	int targetPulseCount = (int) ( (distance / circumferenceInInches) * pulsesPerRotation) ;
    	SmartDashboard.putNumber("Current Position Left", currentPositionLeft);
      	SmartDashboard.putNumber("Current Position Right", currentPositionRight);
      	 SmartDashboard.putNumber("Target position ", targetPulseCount);
    	//encoderTalonLeft.set(ControlMode.Position, targetPulseCount);
     	encoderTalonRight.set(ControlMode.Position, targetPulseCount);
    }
    
    public void initGyro() {
		if (pigeon == null) {
			pigeon = new PigeonIMU (rearRightMotor);
		} 

    		pigeon.setFusedHeading(0.0,10); 
		

		    int loops = 0;
		    do  {
			    pigeon.getGeneralStatus(genStatus);	    
			    SmartDashboard.putString("Starting Pigeon State", genStatus.state.name());
			    Timer.delay(0.01);
		    } while (genStatus.state != PigeonIMU.PigeonState.Ready || loops++ < 3);

    		pigeon.getFusedHeading(fusionStatus);

			startingAngle = fusionStatus.heading;

    	
	    //startingAngle = getHeadingAngle();
	    SmartDashboard.putString("Finished Pigeon Init", genStatus.state.name());

    }
    
    public boolean hasDrivenFarEnough(double distance_in_inches) { 
    	if (encoderTalonLeft != null) {
    		double currentPosition = -encoderTalonLeft.getSensorCollection().getQuadraturePosition() - startingPosition ;
    	
    		int targetPulseCount = (int) ((distance_in_inches / circumferenceInInches) * pulsesPerRotation);
	    	SmartDashboard.putNumber("Current Position ", currentPosition);
	    	SmartDashboard.putNumber("Starting Position ", startingPosition);
	    	SmartDashboard.putNumber("Target position", targetPulseCount);
	    	if (currentPosition >= targetPulseCount) {
	    		return true;
	    	}
	    	return false;
    	} else return false;
    }

    	
    public void tankDrive (double leftSpeed, double rightSpeed) {
		SmartDashboard.putNumber("Left Motors", leftSpeed);
 		SmartDashboard.putNumber("Right Motors", rightSpeed);
    	myRobot.tankDrive(leftSpeed*defaultSpeed,rightSpeed*defaultSpeed);
    	
    }
    public void arcadeDrive (double speed, double angle) {
		SmartDashboard.putNumber("Drive Speed", speed);
 		SmartDashboard.putNumber("Drive Angle", angle);
 		
    	myRobot.arcadeDrive(speed,angle, false); // turn off the squared
    	
    }


    public void forward () {
		SmartDashboard.putString("Drive System", "Going Forwards");
    	myRobot.arcadeDrive(defaultSpeed, 0.0);
    	
    }
    public double distanceUntilObject () {
    	// Returns smaller of left/right distance
		SmartDashboard.putString("Drive System", "Going Forwards");
		
		double distanceLeft =  distanceUntilObjectLeft();
		double distanceRight =  distanceUntilObjectRight();

		if (distanceLeft < 0 && distanceRight > 0) {
			return distanceRight;
		} else if (distanceLeft > 0 && distanceRight < 0) {
			return distanceLeft;
		}
    	return (Math.min(distanceLeft,distanceRight));	   	
    }
    

    public double distanceUntilObjectLeft () {

    	if (ultraLeft !=null) {
        	double distanceLeft = ultraLeft.getRangeInches();
    		SmartDashboard.putNumber("Distance Left Until Object ", distanceLeft );
    		SmartDashboard.putString("Ultra Left Enabled",ultraLeft.isEnabled() ? "Yes" : "No" );
    		SmartDashboard.putString("Ultra Left Valid Range",ultraLeft.isRangeValid() ? "Yes" : "No" );
    		return distanceLeft;
      	} else {
    		SmartDashboard.putString("Ultra Right Enabled","Not initialized");
          	return -1.0; 
    	}
		   	
    }

    public double distanceUntilObjectRight () {

    	if (ultraRight !=null) {

        	double distanceRight = ultraRight.getRangeInches();
    		SmartDashboard.putNumber("Distance Right Until Object ", distanceRight );
    		SmartDashboard.putString("Ultra Right Enabled",ultraRight.isEnabled() ? "Yes" : "No" );
    		SmartDashboard.putString("Ultra Right Valid Range",ultraRight.isRangeValid() ? "Yes" : "No" );
    		return (distanceRight);
    	} else {
    		SmartDashboard.putString("Ultra Right Enabled","Not initialized");
        	return -1.0; 
    	}
		   	
    }
    public void forwardPartialPower (double power) {
		SmartDashboard.putString("Drive System", "Going Forwards Partial Power");
		
		if (power > 1.0) {
			power = 1.0;
		} else if (power < 0.0) {
			power = 0.0;
		}
		
		power = defaultSpeed*power;
		power = Math.max (0.5, power);
    	myRobot.arcadeDrive(defaultSpeed*power, 0.0);
    	
    }

    public double getHeadingAngle() {
	    if (pigeon != null) {
	    	pigeon.getFusedHeading(fusionStatus);
			SmartDashboard.putNumber("Current Angle", fusionStatus.heading);
		    return fusionStatus.heading - startingAngle;
	    } else return 0.0;
	    
 
    }
    
    public void forwardwithGyro () {
    	forwardwithGyro (defaultSpeed);
    }
    public void forwardwithGyro (double speed) {     
    	final double KP = -0.0003; // Constant for how fast to driving angle
    	double currentAngle = 0.0;
    	boolean pigeonIsGood = true;
    	if ( pigeon != null) {
			SmartDashboard.putString("D rive System", "Gyro Going Forwards");
		    pigeon.getFusedHeading(fusionStatus);
		    pigeon.getGeneralStatus(genStatus);	
		    SmartDashboard.putString("Pigeon Error",  genStatus.lastError.name());
		    SmartDashboard.putString("Pigeon Booting",  genStatus.bCalIsBooting ? "True" : "False");
		    SmartDashboard.putString("Pigeon State", genStatus.state.name());
			
		     currentAngle = fusionStatus.heading; 
		    SmartDashboard.putString("Pigeon State", pigeon.getState().name());
		    
		    pigeonIsGood = (pigeon.getState() == PigeonIMU.PigeonState.Ready);
    	} else pigeonIsGood = false;
    	
    	if (!pigeonIsGood) {		
		
    		arcadeDrive(speed, 0);
    	}  
		else {
			double correction=PID_Gyro_Function(startingAngle,currentAngle); // What is the correction to get the current angle closer to the starting angle
			SmartDashboard.putNumber("Correction", correction);
			
			tankDrive(speed - 0.3 * (correction / 90),speed + 0.5 * (correction / 90));
		}
    	
	SmartDashboard.putNumber("Current Angle", currentAngle);
	SmartDashboard.putNumber("Starting Angle", startingAngle);
	SmartDashboard.putString("PigeonIsGood", (pigeonIsGood ? "Yes" :"No") );	
    }
	
    public void backward() {
		SmartDashboard.putString("Drive System", "Going Backwards");
    	myRobot.tankDrive(-defaultSpeed, -defaultSpeed);
    }

	//vars for PID_Gyro_Function
	double P = 1;
	double I= 0.5;
	double D = 0.1;
	
    double integral=0;
	double prev_error = 0;

    public double PID_Gyro_Function(double target,double currentAngleHeading){
	
		double error = target - currentAngleHeading; // Error = Target - Actual
        integral += (error*.02); // Integral is increased by the error*time 
        double derivative = (error - prev_error) / .02;
        double correction = P*error + I*integral + D*derivative;//change P, I ,D at the the top 
		prev_error=error;
		return correction;
	}
    public void turnLeft() {
		SmartDashboard.putString("Drive System", "Turning Left");
    	myRobot.tankDrive(-defaultTurnSpeed, defaultTurnSpeed);
    }
    
    public void turnLeft(double power) {
		SmartDashboard.putString("Drive System", "Turning Left");
		power = Math.max(0.55, power);
		SmartDashboard.putNumber("Turning Power", power);
    	myRobot.tankDrive(-defaultTurnSpeed*power,defaultTurnSpeed*power);
    }
    
    
    public void turnRight() {
		SmartDashboard.putString("Drive System", "Turning Right");
    	myRobot.tankDrive(defaultTurnSpeed, -defaultTurnSpeed);
    }
    
    public void turnRight(double power) {
		SmartDashboard.putString("Drive System", "Turning Right");
		power = Math.max(0.55, power);
		SmartDashboard.putNumber("Turning Power", power);
		
    	myRobot.tankDrive(defaultTurnSpeed*power,-defaultTurnSpeed*power);
    }
    
    public void stop() {
		SmartDashboard.putString("Drive System", "Stopping");
    	myRobot.tankDrive(0, 0);
    }
    
}
