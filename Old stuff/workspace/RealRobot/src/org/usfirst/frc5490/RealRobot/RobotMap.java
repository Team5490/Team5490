// RobotBuilder Version: 1.5
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package org.usfirst.frc5490.RealRobot;
    
import edu.wpi.first.wpilibj.*;

import edu.wpi.first.wpilibj.livewindow.LiveWindow;
//import java.util.Vector;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    public static SpeedController driveTrainLeftFront;
    public static SpeedController driveTrainLeftRear;
    public static SpeedController driveTrainRightFront;
    public static SpeedController driveTrainRightRear;
    public static RobotDrive driveTrainRobotDrive41;
    public static SpeedController liftLiftMotor;
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    public static void init() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        driveTrainLeftFront = new Talon(0);                                               // check port #s *****
        LiveWindow.addActuator("Drive Train", "Left Front", (Talon) driveTrainLeftFront);
        
        driveTrainLeftRear = new Talon(1);                                               // check port #s *****
        LiveWindow.addActuator("Drive Train", "Left Rear", (Talon) driveTrainLeftRear);
        
        driveTrainRightFront = new Talon(2);                                               // check port #s *****
        LiveWindow.addActuator("Drive Train", "Right Front", (Talon) driveTrainRightFront);
        
        driveTrainRightRear = new Talon(4);                                               // check port #s *****
        LiveWindow.addActuator("Drive Train", "Right Rear", (Talon) driveTrainRightRear);
        
        driveTrainRobotDrive41 = new RobotDrive(driveTrainLeftFront, driveTrainLeftRear,
              driveTrainRightFront, driveTrainRightRear);
        
        driveTrainRobotDrive41.setSafetyEnabled(true);
        driveTrainRobotDrive41.setExpiration(0.1);
        driveTrainRobotDrive41.setSensitivity(0.5);
        driveTrainRobotDrive41.setMaxOutput(1.0);

        liftLiftMotor = new Talon(3);
        LiveWindow.addActuator("Lift", "Lift Motor", (Talon) liftLiftMotor);
        
        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
    }
}
