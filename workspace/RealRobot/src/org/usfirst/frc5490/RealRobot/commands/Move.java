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

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc5490.RealRobot.Robot;

import  org.usfirst.frc5490.RealRobot.subsystems.driveTrain;

/**
 *
 */
public class  Move extends Command {


    public Move() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);

        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
        requires(Robot.driveTrain);
        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {

        drive left = new drive();
            double driveLeft = left.leftMotor();       //retrieve left and right motor speeds from DriveTrain
        drive right = new drive();
            double driveRight = right.rightMotor();

        double driveTotal = driveLeft + driveRight;     //total motor output used to test if robot is moving

        if (driveTotal > 0) {                           //robot is moving, so set motors to speed
        RobotMap.driveTrainLeftFront.set(driveLeft);
        RobotMap.driveTrainLeftRear.set(driveLeft);
        RobotMap.driveTrainRightFront.set(-driveRight); //right motors go in opposite direction
        RobotMap.driveTrainRightFront.set(-driveRight);
    }   else {
        RobotMap.driveTrainLeftFront.set(0);            //set motors to zero if robot is not moving
        RobotMap.driveTrainLeftRear.set(0);
        RobotMap.driveTrainRightFront.set(0);
        RobotMap.driveTrainRightFront.set(0);
    }
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
