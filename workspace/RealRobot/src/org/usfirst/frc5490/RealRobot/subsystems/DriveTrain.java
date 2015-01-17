// RobotBuilder Version: 1.5
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package org.usfirst.frc5490.RealRobot.subsystems;

import org.usfirst.frc5490.RealRobot.RobotMap;
import org.usfirst.frc5490.RealRobot.commands.*;
import edu.wpi.first.wpilibj.*;

import edu.wpi.first.wpilibj.command.Subsystem;

import org.usfirst.frc5490.RealRobot.OI;


/**
 *
 */
public class DriveTrain extends Subsystem {
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    SpeedController leftFront = RobotMap.driveTrainLeftFront;   //declaration of motors
    SpeedController leftRear = RobotMap.driveTrainLeftRear;
    SpeedController rightFront = RobotMap.driveTrainRightFront;
    SpeedController rightRear = RobotMap.driveTrainRightRear;
    RobotDrive robotDrive41 = RobotMap.driveTrainRobotDrive41;
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

public void drive()  {  //drive method

    OI joystickPosition = new OI();
    double ySpeed = joystickPosition.getYValue();   //retrieves joystick y from OI.java

    private double driveSpeed;
    this.driveSpeed = ySpeed^3; //makes motor output cubic

    /*public double getDriveSpeed()   {
        return this.driveSpeed;
    }   */  //not needed

    //if (driveSpeed > 0.8)   {
        //driveSpeed = 0.8;
    //}
    //if (driveSpeed < -0.8)    {
        //driveSpeed = -0.8;
    //}                             ////limits motor output to ± 8
}
    if (driveSpeed < 0.08 and driveSpeed > 0)  {    //sets anything within ± 0.08 to zero...
        driveSpeed = 0;                             //...which is used to make turning in place easier
    }
    if (driveSpeed > -0.08 and driveSpeed < 0) {
        driveSpeed = 0;
    }

    OI joystickPosition = new OI();
    double xSteer = joystickPosition.getXValue();   //retrieves joystick x value from OI.java

    private double leftSpeed = driveSpeed;  //direct motor outputs for left and right sides
    private double rightSpeed = driveSpeed;

    if (xSteer > 1) {                               //steering functions
        this.leftSpeed = leftSpeed + xSteer/6
        this.rightSpeed = rightSpeed - xSteer/6
    }
    if (xSteer < 1) {
        this.leftSpeed = leftSpeed - xSteer/6
        this.rightSpeed = rightSpeed + xSteer/6
    }

    if (xSteer > 1 and driveSpeed = 0)  {
        this.leftSpeed = 0.3;
        this.rightSpeed = -0.3;
    }
    if (xSteer < 1 and driveSpeed = 0)  {
        this.leftSpeed = -0.3;
        this.rightSpeed = 0.3;
    }                                               //end steering functions

    public double leftMotor()   {
        return this.leftSpeed;      //sets public variables for use in Move.java (command output)
    }
    public double rightMotor()  {
        return this.rightSpeed;
    }


}

    
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void initDefaultCommand() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND
        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND
	   setDefaultCommand(new Move());                                            //Move.java is command output
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
}

