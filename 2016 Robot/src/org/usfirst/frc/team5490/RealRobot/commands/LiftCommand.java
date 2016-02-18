// RobotBuilder Version: 1.5
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package org.usfirst.frc5490.RealRobot.commands;

//import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

import org.usfirst.frc5490.RealRobot.Robot;
//import org.usfirst.frc5490.RealRobot.RobotMap;
import org.usfirst.frc5490.RealRobot.RobotMap;


//import edu.wpi.first.wpilibj.Timer.*;
/*
import edu.wpi.first.wpilibj.Timer.start;
import edu.wpi.first.wpilibj.Timer.stop;
import edu.wpi.first.wpilibj.Timer.reset;
import edu.wpi.first.wpilibj.Timer.get;

/**
 *
 */
public class  LiftCommand extends Command {

    public LiftCommand() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);

        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
        requires(Robot.lift);
        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Timer liftTime;
    	
        liftTime = new Timer();
        liftTime.start();

        double timePassed = liftTime.get();

        while (timePassed < 5)  {   //arbitrary number *
            RobotMap.liftLiftMotor.set(0.5);    //arbitrary number *
            timePassed = liftTime.get();
        }
        RobotMap.liftLiftMotor.disable();
        liftTime.stop();
        liftTime.reset();
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return true;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
