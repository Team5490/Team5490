package org.usfirst.frc.team5490.robot;


import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Timer;

/**
 * This sample program shows how to control a motor using a joystick. In the operator
 * control part of the program, the joystick is read and the value is written to the motor.
 *
 * Joystick analog values range from -1 to 1 and speed controller inputs also range from -1
 * to 1 making it easy to work together. The program also delays a short time in the loop
 * to allow other threads to run. This is generally a good idea, especially since the joystick
 * values are only transmitted from the Driver Station once every 20ms.
 */
public class Robot extends SampleRobot {
	
    private SpeedController motor0;	// the motor to directly control with a joystick
    private SpeedController motor1; 
    private SpeedController motor2; // Once we get front and back sorted out we should label them like 'left1', 'left2', etc.
    private SpeedController motor3;
    private Joystick stick;

	private final double k_updatePeriod = 0.005; // update every 0.005 seconds/5 milliseconds (200Hz)

    public Robot() {
        motor0 = new Talon(0);		// initialize the motor as a Talon on channel 0 on rio
        motor1 = new Talon(1);      // channel 1
        motor2 = new Talon(2);      // etc
        motor3 = new Talon(3);
        stick0 = new Joystick(0);	// initialize the joystick on port 0
//        stick1 = new Joystick(1); //We have yet to work our code around two joysticks but until then we leave this for testing?
        
         /**
          * Problem we could probably fix: joystick is incredibly ambiguous
          * and truth be told idk how to fix it. If the xbox controller
          * is in port 0, it uses the left stick. If the joystick is in port zero, it uses that.
          * Either we find a way to assure they are always in the right ports, or find something 
          * else.
          * Also, how will we end up working both joysticks on the 360 controller?
          * ~~the world will never know~~
          */
    }

    /**
     * Runs the motor from a joystick.
     */
    public void operatorControl() {
        while (isOperatorControl() && isEnabled()) {
        	// Set the motor's output.
        	// This takes a number from -1 (100% speed in reverse) to +1 (100% speed going forward)
        	motor0.set(stick.getY());
        	motor1.set(stick.getY());
        	motor2.set(-1 * (stick.getY()));      //this fix is a crapshoot
        	motor3.set(-1 *(stick.getY()));       //set all to stick.getY()) for rave mode party time
        	
            Timer.delay(k_updatePeriod);	// wait 5ms to the next update
        }
        
    }
}
