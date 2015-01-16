
package org.usfirst.frc.team5490.robot;


import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;

public class Robot extends SampleRobot {
    RobotDrive myRobot;
    Joystick stick;

    public Robot() {
        myRobot = new RobotDrive(1, 1, 1, 1);
        myRobot.setExpiration(0.1);
        stick = new Joystick(0);
    }

   
    public void autonomous() {
        myRobot.setSafetyEnabled(false);
        myRobot.drive(-0.5, 0.0);	// drive forwardsssss plox plox
        Timer.delay(2.0);		//    for 2 sex
        myRobot.drive(0.0, 0.0);	// stop mlady
    }

    public void operatorControl() {
        myRobot.setSafetyEnabled(true);
        while (isOperatorControl() && isEnabled()) {
            myRobot.arcadeDrive(stick); // roght stick m8
            Timer.delay(0.005);		// wait m99999
        }
    }

    public void test() {
    }
}
