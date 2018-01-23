package org.usfirst.frc.team6873.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class Autonomous1Right extends CommandGroup {

    public Autonomous1Right() {
        // e.g. addSequential(new Command1());
        //      addSequential(new Command2());
        // these will run in order.

        // To run multiple commands at the same time,
        // use addParallel()
        // e.g. addParallel(new Command1());
        //      addSequential(new Command2());
        // Command1 and Command2 will run in parallel.

        // A command group will require all of the subsystems that each member
        // would require.
        // e.g. if Command1 requires chassis, and Command2 requires arm,
        // a CommandGroup containing them would require both the chassis and the
        // arm.

    	addSequential (new DriveForward(5));
    	addSequential (new TurnRight(90));
    	addSequential (new DriveForward(6));
    	addSequential (new TurnLeft(90));
    	addSequential (new DriveForward(11));
    	addSequential (new TurnLeft(90));
    	addSequential (new DriveForward(1));
    
    }
}
