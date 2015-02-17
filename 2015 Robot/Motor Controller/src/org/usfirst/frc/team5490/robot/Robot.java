
package org.usfirst.frc.team5490.robot;


import java.util.Comparator;
import java.util.Vector;

import com.ni.vision.NIVision;
import com.ni.vision.NIVision.DrawMode;
import com.ni.vision.NIVision.Image;
import com.ni.vision.NIVision.ImageType;
import com.ni.vision.NIVision.ShapeMode;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Robot extends SampleRobot {
	
	int session;
	Image binaryFrame;
	int imaqError;
	Image frame;
	Image frame2;
	CameraServer camServer;
    private SpeedController motor0;     //Defining everything
    private SpeedController motor1;
    private DoubleSolenoid solenoid1;
    private SpeedController snowmotor0;
    private SpeedController snowmotor1;
    private Joystick stick;
    private Joystick xbox;
    NIVision.Range TOTE_HUE_RANGE = new NIVision.Range(24, 49);	//Default hue range for yellow tote
	NIVision.Range TOTE_SAT_RANGE = new NIVision.Range(67, 255);	//Default saturation range for yellow tote
	NIVision.Range TOTE_VAL_RANGE = new NIVision.Range(49, 255);	//Default value range for yellow tote
	
	NIVision.ParticleFilterCriteria2 criteria[] = new NIVision.ParticleFilterCriteria2[1];
	NIVision.ParticleFilterOptions2 filterOptions = new NIVision.ParticleFilterOptions2(0,0,1,1);

	
	double AREA_MINIMUM = 0.5; //Default Area minimum for particle as a percentage of total image area
	double LONG_RATIO = 2.22; //Tote long side = 26.9 / Tote height = 12.1 = 2.22
	double SHORT_RATIO = 1.4; //Tote short side = 16.9 / Tote height = 12.1 = 1.4
	double SCORE_MIN = 75.0;  //Minimum score to be considered a tote
	double VIEW_ANGLE = 49.4; //View angle fo camera, set to Axis m1011 by default, 64 for m1013, 51.7 for 206, 52 for HD3000 square, 60 for HD3000 640x480

	Scores scores = new Scores();
	//Just changing this for now, used to be 0.005
	private final double k_updatePeriod = 0.0005;
	
	public class ParticleReport implements Comparator<ParticleReport>, Comparable<ParticleReport>{
		double PercentAreaToImageArea;
		double Area;
		double ConvexHullArea;
		double BoundingRectLeft;
		double BoundingRectTop;
		double BoundingRectRight;
		double BoundingRectBottom;
		
		public int compareTo(ParticleReport r)
		{
			return (int)(r.Area - this.Area);
		}
		
		public int compare(ParticleReport r1, ParticleReport r2)
		{
			return (int)(r1.Area - r2.Area);
		}
	};

	//Structure to represent the scores for the various tests used for target identification
	public class Scores {
		double Trapezoid;
		double LongAspect;
		double ShortAspect;
		double AreaToConvexHullArea;
	};

    public Robot() {
    	frame = NIVision.imaqCreateImage(NIVision.ImageType.IMAGE_RGB, 0);
    	frame2 = NIVision.imaqCreateImage(NIVision.ImageType.IMAGE_RGB, 0);
		binaryFrame = NIVision.imaqCreateImage(ImageType.IMAGE_U8, 0);
		criteria[0] = new NIVision.ParticleFilterCriteria2(NIVision.MeasurementType.MT_AREA_BY_IMAGE_AREA, AREA_MINIMUM, 100.0, 0, 0);
		
		
		//SmartDashboard.putNumber("Tote hue min", TOTE_HUE_RANGE.minValue);
		//SmartDashboard.putNumber("Tote hue max", TOTE_HUE_RANGE.maxValue);
		//SmartDashboard.putNumber("Tote sat min", TOTE_SAT_RANGE.minValue);
		//SmartDashboard.putNumber("Tote sat max", TOTE_SAT_RANGE.maxValue);
		//SmartDashboard.putNumber("Tote val min", TOTE_VAL_RANGE.minValue);
		//SmartDashboard.putNumber("Tote val max", TOTE_VAL_RANGE.maxValue);
		//SmartDashboard.putNumber("Area min %", AREA_MINIMUM);

    	session = NIVision.IMAQdxOpenCamera("cam0", NIVision.IMAQdxCameraControlMode.CameraControlModeController);
    	//camServer = CameraServer.getInstance();
    	//camServer.setQuality(50);
    	//camServer.startAutomaticCapture("cam0");
    	NIVision.IMAQdxConfigureGrab(session);
    	
    	
		

		//Defining the controllers, both motor controllers and HID controllers
		
        motor0 = new Talon(0);
        motor1 = new Talon(1);
        snowmotor0 = new Talon(2);
        snowmotor1 = new Talon(3);
        stick = new Joystick(0);   //Joystick, needs to be in port 0
        xbox = new Joystick(1);    //360 Controller, needs to be in port 1
                                   //If these get backwards the robot won't act. They can be dragged in driver station
        
        solenoid1 = new DoubleSolenoid(0,1);  //Defining the solenoid.
    }
    public void autonomous() {
    	// BELOW IS AUTONOMOUS, CHANGE BACK LATER
		//while (isOperatorControl() && isEnabled())
    	while (isAutonomous() && isEnabled())
		{
    		motor0.set(-0.3);
    		motor1.set(0.3);             //This stuff we fix tomorrow (2/16)
    		Timer.delay(2.0);
    		motor0.set(0);
    		motor1.set(0);
			//read file in from disk. For this example to run you need to copy image20.jpg from the SampleImages folder to the
			//directory shown below using FTP or SFTP: http://wpilib.screenstepslive.com/s/4485/m/24166/l/282299-roborio-ftp
			//NIVision.imaqReadFile(frame, "/home/lvuser/SampleImages/image20.jpg");
			NIVision.IMAQdxGrab(session, frame, 1);

			//Update threshold values from SmartDashboard. For performance reasons it is recommended to remove this after calibration is finished.
			TOTE_HUE_RANGE.minValue = (int)SmartDashboard.getNumber("Tote hue min", TOTE_HUE_RANGE.minValue);
			TOTE_HUE_RANGE.maxValue = (int)SmartDashboard.getNumber("Tote hue max", TOTE_HUE_RANGE.maxValue);
			TOTE_SAT_RANGE.minValue = (int)SmartDashboard.getNumber("Tote sat min", TOTE_SAT_RANGE.minValue);
			TOTE_SAT_RANGE.maxValue = (int)SmartDashboard.getNumber("Tote sat max", TOTE_SAT_RANGE.maxValue);
			TOTE_VAL_RANGE.minValue = (int)SmartDashboard.getNumber("Tote val min", TOTE_VAL_RANGE.minValue);
			TOTE_VAL_RANGE.maxValue = (int)SmartDashboard.getNumber("Tote val max", TOTE_VAL_RANGE.maxValue);

			//Threshold the image looking for yellow (tote color)
			NIVision.imaqColorThreshold(binaryFrame, frame, 255, NIVision.ColorMode.HSV, TOTE_HUE_RANGE, TOTE_SAT_RANGE, TOTE_VAL_RANGE);

			//Send particle count to dashboard
			int numParticles = NIVision.imaqCountParticles(binaryFrame, 1);
			SmartDashboard.putNumber("Masked particles", numParticles);

			//Send masked image to dashboard to assist in tweaking mask.
			CameraServer.getInstance().setImage(frame);

			//filter out small particles
			float areaMin = (float)SmartDashboard.getNumber("Area min %", AREA_MINIMUM);
			criteria[0].lower = areaMin;
			imaqError = NIVision.imaqParticleFilter4(binaryFrame, binaryFrame, criteria, filterOptions, null);

			//Send particle count after filtering to dashboard
			numParticles = NIVision.imaqCountParticles(binaryFrame, 1);
			SmartDashboard.putNumber("Filtered particles", numParticles);
			
			
			if(numParticles > 0)
			{
				//Measure particles and sort by particle size
				Vector<ParticleReport> particles = new Vector<ParticleReport>();
				for(int particleIndex = 0; particleIndex < numParticles; particleIndex++)
				{
					ParticleReport par = new ParticleReport();
					par.PercentAreaToImageArea = NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_AREA_BY_IMAGE_AREA);
					par.Area = NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_AREA);
					par.ConvexHullArea = NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_CONVEX_HULL_AREA);
					par.BoundingRectTop = NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_BOUNDING_RECT_TOP);
					par.BoundingRectLeft = NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_BOUNDING_RECT_LEFT);
					par.BoundingRectBottom = NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_BOUNDING_RECT_BOTTOM);
					par.BoundingRectRight = NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_BOUNDING_RECT_RIGHT);
					particles.add(par);
				}
				particles.sort(null);

				//This example only scores the largest particle. Extending to score all particles and choosing the desired one is left as an exercise
				//for the reader. Note that the long and short side scores expect a single tote and will not work for a stack of 2 or more totes.
				//Modification of the code to accommodate 2 or more stacked totes is left as an exercise for the reader.
                //Well gee thanks wpilib
				scores.Trapezoid = TrapezoidScore(particles.elementAt(0));
				SmartDashboard.putNumber("Trapezoid", scores.Trapezoid);
				scores.LongAspect = LongSideScore(particles.elementAt(0));
				SmartDashboard.putNumber("Long Aspect", scores.LongAspect);
				scores.ShortAspect = ShortSideScore(particles.elementAt(0));
				SmartDashboard.putNumber("Short Aspect", scores.ShortAspect);
				scores.AreaToConvexHullArea = ConvexHullAreaScore(particles.elementAt(0));
				SmartDashboard.putNumber("Convex Hull Area", scores.AreaToConvexHullArea);
				boolean isTote = scores.Trapezoid > SCORE_MIN && (scores.LongAspect > SCORE_MIN || scores.ShortAspect > SCORE_MIN) && scores.AreaToConvexHullArea > SCORE_MIN;
				boolean isLong = scores.LongAspect > scores.ShortAspect;
				
				//Send distance and tote status to dashboard. The bounding rect, particularly the horizontal center (left - right) may be useful for rotating/driving towards a tote
				SmartDashboard.putBoolean("IsTote", isTote);
				SmartDashboard.putNumber("Distance", computeDistance(binaryFrame, particles.elementAt(0), isLong));

			} else {
				SmartDashboard.putBoolean("IsTote", false);
			}
			/*motor0.set(-0.3);
    		motor1.set(0.3);
    		Timer.delay(2.0);
    		motor0.set(0);
    		motor1.set(0); */
			
			Timer.delay(0.005);				// wait for a motor update time
		
		
		}
    	//NIVision.IMAQdxStopAcquisition(session);
    }

    
    public void operatorControl() {       //teleop
    	
    	NIVision.IMAQdxStartAcquisition(session);
    	NIVision.Rect rect = new NIVision.Rect(10, 10, 100, 100);
    	
    	
    	
        while (isOperatorControl() && isEnabled()) {
        	int value = 0;
        	NIVision.IMAQdxGrab(session, frame2, 1);
        	NIVision.imaqDrawShapeOnImage(frame2, frame2, rect, DrawMode.DRAW_VALUE, ShapeMode.SHAPE_OVAL, 0.0f); //most important line of code obvs
        	CameraServer.getInstance().setImage(frame2);
        	
        	
        	
        	//SmartDashboard.putDouble("randomword", stick.getY());
        	//SmartDashboard.putDouble("randomword2", stick.getZ());
        	
        	//double zNegativeBlockone = -0.2;
        	//if (stick.getY() > 0.1) {
        	//	zNegativeBlockone = -0.5;
        	//}
        		
        	/*double OIy = stick.getY();
        	double OIz = stick.getZ();
        	double OIthrottle = stick.getThrottle();
        	
        	double throttleValue = -2*OIthrottle+3;
        	
        	double driveSpeed = Math.pow(OIy, throttleValue);
        	double xSteer = Math.pow(OIz, throttleValue);
        	
        	double leftSpeed = driveSpeed;
        	double rightSpeed = driveSpeed;
        	
        	if (xSteer > 0.05 && driveSpeed < 0.2 && driveSpeed > -0.2)	{
        		leftSpeed = leftSpeed + xSteer/8;
        	}
        	if (xSteer < 0.05 && driveSpeed < 0.2 && driveSpeed > -0.2)	{
        		rightSpeed = rightSpeed + xSteer/8;
        	}
        	
          	if (xSteer > 0.05 && Math.abs(driveSpeed) > 0.2)	{
        		leftSpeed = leftSpeed + xSteer/10;
        		rightSpeed = rightSpeed - xSteer/10;
        	}
        	if (xSteer < 0.05 && Math.abs(driveSpeed) > 0.2)	{
        		rightSpeed = rightSpeed + xSteer/10;
        		leftSpeed = leftSpeed - xSteer/10;
        	}
        	
        	if (xSteer > 0.4 && driveSpeed < 0.2 && driveSpeed > -0.2)	{
        		leftSpeed = xSteer/3;
        		rightSpeed = -1*xSteer/3;
        	}
        	if (xSteer > 0.4 && driveSpeed < 0.2 && driveSpeed > -0.2)	{
        		leftSpeed = -1*xSteer/3;
        		rightSpeed = xSteer/3;
        	}
        	
        	
        	if (leftSpeed > 1)	{
        		leftSpeed = 0.99;
        	}
        	if (rightSpeed > 1)	{
        		rightSpeed = 0.99;
        	}
        	
        	
        	motor0.set(-1*(-1*leftSpeed));
        	motor1.set(-1*rightSpeed);*/
        	
        	
        	//test buttons
        	boolean button1 = stick.getRawButton(11);
        	boolean button2 = stick.getRawButton(12);
        	boolean button3 = xbox.getRawButton(7);
        	boolean button4 = xbox.getRawButton(8);
        	double ltrigger = xbox.getRawAxis(2);
        	double rtrigger = xbox.getRawAxis(3);
            boolean xbutton1 = xbox.getRawButton(1);     // A Button on 360 controller
            boolean xbutton2 = xbox.getRawButton(2);     // Y Button on 360 controller
        	
        	//just for testing
            //idk what would work better; analog triggers or just button on/off. Buttons seem to work tho.
        	SmartDashboard.putDouble("whocares", ltrigger);
        	SmartDashboard.putDouble("whocares", rtrigger);
        	
            //Lift
            //These must always be set with one negative and the other positive so we don't destroy the motors
            
            if(button1) //Up when button 11 is held. Probably going to change this.
        		{
        		snowmotor0.set(1.0);
        		snowmotor1.set(-1);
        		}
        		
        	else if(button2) //Down
        	{
        		snowmotor0.set(-0.3);
        		snowmotor1.set(0.3);
        	}
        	else	         //Hold when nothing is held
        		{
        		snowmotor0.set(0);
        		snowmotor1.set(0);
        		}
        	
        	//Below here is for the analog trigger
            //I was going to implement this, but because
        	/*
        	if(ltrigger > 1) //Up
    		{
    		snowmotor0.set(1.0);
    		snowmotor1.set(-1);
    		}
    		
        	else if(rtrigger > 1) //Down
        	{
    		snowmotor0.set(-0.3);
    		snowmotor1.set(0.3);
        	}
        	else if((rtrigger > 1)&&(ltrigger > 1))
    		{
    		snowmotor0.set(0);
    		snowmotor1.set(0);
    		}
            */
        	/*if(button2)
        		{snowmotor1.set(.3);}
        	else
        		{snowmotor1.set(0);}
        	*/
        	
        	if(button3)
        		{solenoid1.set(DoubleSolenoid.Value.kForward);}         //Start activates the solenoid
        	else if(button4)
        		{solenoid1.set(DoubleSolenoid.Value.kReverse);}         //Back puts it the other way
        	//else
    		//	{solenoid1.set(DoubleSolenoid.Value.kOff);}       We don't need it to be off I guess?
        		
        	
        	
        	double z = stick.getZ();
        	double y = stick.getY();
        	double throttle = stick.getThrottle();    //The little slider thing on the joystick
        	
        	//spdLimit uses the throttle bar to reduce the max speed of the robot and allow finer control
        	double spdLimit = (3 - throttle) / 4; //converts throttle range from -1,1 to 2,4
        		
        	//if (stick.getZ() < zNegativeBlockone || stick.getZ() > 0.1) {
        	motor0.set(spdLimit * Math.pow((y+z), 3)); //back left
        		//motor1.set(-1*(stick.getY() - stick.getZ()) * (stick.getY() - stick.getZ()) * (stick.getY() - stick.getZ())); // front left
        		//motor2.set((stick.getY() + stick.getZ()) * (stick.getY() + stick.getZ()) * (stick.getY() + stick.getZ())); // front right
        		//motor3.set((stick.getY() + stick.getZ()) * (stick.getY() + stick.getZ()) * (stick.getY() + stick.getZ())); // back right
        	motor1.set(-1 * spdLimit * Math.pow((y-z), 3));
        				//}
        	 
        	 
        	//else {
        	//	motor0.set(-1*stick.getY());
        	//	motor1.set(-1*stick.getY());
        		//motor2.set(stick.getY());
        		//motor3.set(stick.getY());
        	
        	//}
        	//There used to be vision controls here, but it lagged the driving like crazy because 
        	//of how much info it did in the loop 
        	
        	
        	//
        	//NIVision.IMAQdxGrab(session, frame, 1);
        	/*if (value == 0) {
				//Update threshold values from SmartDashboard. For performance reasons it is recommended to remove this after calibration is finished.
				TOTE_HUE_RANGE.minValue = (int)SmartDashboard.getNumber("Tote hue min", TOTE_HUE_RANGE.minValue);
				TOTE_HUE_RANGE.maxValue = (int)SmartDashboard.getNumber("Tote hue max", TOTE_HUE_RANGE.maxValue);
				TOTE_SAT_RANGE.minValue = (int)SmartDashboard.getNumber("Tote sat min", TOTE_SAT_RANGE.minValue);
				TOTE_SAT_RANGE.maxValue = (int)SmartDashboard.getNumber("Tote sat max", TOTE_SAT_RANGE.maxValue);
				TOTE_VAL_RANGE.minValue = (int)SmartDashboard.getNumber("Tote val min", TOTE_VAL_RANGE.minValue);
				TOTE_VAL_RANGE.maxValue = (int)SmartDashboard.getNumber("Tote val max", TOTE_VAL_RANGE.maxValue);
	
				//Threshold the image looking for yellow (tote color)
				NIVision.imaqColorThreshold(binaryFrame, frame, 255, NIVision.ColorMode.HSV, TOTE_HUE_RANGE, TOTE_SAT_RANGE, TOTE_VAL_RANGE);
	
				//Send particle count to dashboard
				int numParticles = NIVision.imaqCountParticles(binaryFrame, 1);
				SmartDashboard.putNumber("Masked particles", numParticles);
	
				//Send masked image to dashboard to assist in tweaking mask.
				CameraServer.getInstance().setImage(frame);
	
				//filter out small particles
				float areaMin = (float)SmartDashboard.getNumber("Area min %", AREA_MINIMUM);
				criteria[0].lower = areaMin;
				imaqError = NIVision.imaqParticleFilter4(binaryFrame, binaryFrame, criteria, filterOptions, null);
	
				//Send particle count after filtering to dashboard
				numParticles = NIVision.imaqCountParticles(binaryFrame, 1);
				SmartDashboard.putNumber("Filtered particles", numParticles);
				
				
				if(numParticles > 0)
				{
					//Measure particles and sort by particle size
					Vector<ParticleReport> particles = new Vector<ParticleReport>();
					for(int particleIndex = 0; particleIndex < numParticles; particleIndex++)
					{
						ParticleReport par = new ParticleReport();
						par.PercentAreaToImageArea = NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_AREA_BY_IMAGE_AREA);
						par.Area = NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_AREA);
						par.ConvexHullArea = NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_CONVEX_HULL_AREA);
						par.BoundingRectTop = NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_BOUNDING_RECT_TOP);
						par.BoundingRectLeft = NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_BOUNDING_RECT_LEFT);
						par.BoundingRectBottom = NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_BOUNDING_RECT_BOTTOM);
						par.BoundingRectRight = NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_BOUNDING_RECT_RIGHT);
						particles.add(par);
					}
					particles.sort(null);
	
					//This example only scores the largest particle. Extending to score all particles and choosing the desired one is left as an exercise
					//for the reader. Note that the long and short side scores expect a single tote and will not work for a stack of 2 or more totes.
					//Modification of the code to accommodate 2 or more stacked totes is left as an exercise for the reader.
					scores.Trapezoid = TrapezoidScore(particles.elementAt(0));
					SmartDashboard.putNumber("Trapezoid", scores.Trapezoid);
					scores.LongAspect = LongSideScore(particles.elementAt(0));
					SmartDashboard.putNumber("Long Aspect", scores.LongAspect);
					scores.ShortAspect = ShortSideScore(particles.elementAt(0));
					SmartDashboard.putNumber("Short Aspect", scores.ShortAspect);
					scores.AreaToConvexHullArea = ConvexHullAreaScore(particles.elementAt(0));
					SmartDashboard.putNumber("Convex Hull Area", scores.AreaToConvexHullArea);
					boolean isTote = scores.Trapezoid > SCORE_MIN && (scores.LongAspect > SCORE_MIN || scores.ShortAspect > SCORE_MIN) && scores.AreaToConvexHullArea > SCORE_MIN;
					boolean isLong = scores.LongAspect > scores.ShortAspect;
					
					//Send distance and tote status to dashboard. The bounding rect, particularly the horizontal center (left - right) may be useful for rotating/driving towards a tote
					SmartDashboard.putBoolean("IsTote", isTote);
					SmartDashboard.putNumber("Distance", computeDistance(binaryFrame, particles.elementAt(0), isLong));
	
				} else {
					SmartDashboard.putBoolean("IsTote", false);
				}
        	}
			*/
        	//This did something but I removed it and it threw errors so idk
			value++;
			if (value > 100) {
				value = 0;
			}
			value = 5;
        	
            Timer.delay(k_updatePeriod);
        }
        
        
        NIVision.IMAQdxStopAcquisition(session);
        
    }
    static boolean CompareParticleSizes(ParticleReport particle1, ParticleReport particle2)
	{
		//we want descending sort order
		return particle1.PercentAreaToImageArea > particle2.PercentAreaToImageArea;
	}

	/**
	 * Converts a ratio with ideal value of 1 to a score. The resulting function is piecewise
	 * linear going from (0,0) to (1,100) to (2,0) and is 0 for all inputs outside the range 0-2
	 */
	double ratioToScore(double ratio)
	{
		return (Math.max(0, Math.min(100*(1-Math.abs(1-ratio)), 100)));
	}

	/**
	 * Method to score convex hull area. This scores how "complete" the particle is. Particles with large holes will score worse than a filled in shape
	 */
	double ConvexHullAreaScore(ParticleReport report)
	{
		return ratioToScore((report.Area/report.ConvexHullArea)*1.18);
	}

	/**
	 * Method to score if the particle appears to be a trapezoid. Compares the convex hull (filled in) area to the area of the bounding box.
	 * The expectation is that the convex hull area is about 95.4% of the bounding box area for an ideal tote.
	 */
	double TrapezoidScore(ParticleReport report)
	{
		return ratioToScore(report.ConvexHullArea/((report.BoundingRectRight-report.BoundingRectLeft)*(report.BoundingRectBottom-report.BoundingRectTop)*.954));
	}

	/**
	 * Method to score if the aspect ratio of the particle appears to match the long side of a tote.
	 */
	double LongSideScore(ParticleReport report)
	{
		return ratioToScore(((report.BoundingRectRight-report.BoundingRectLeft)/(report.BoundingRectBottom-report.BoundingRectTop))/LONG_RATIO);
	}

	/**
	 * Method to score if the aspect ratio of the particle appears to match the short side of a tote.
	 */
	double ShortSideScore(ParticleReport report){
		return ratioToScore(((report.BoundingRectRight-report.BoundingRectLeft)/(report.BoundingRectBottom-report.BoundingRectTop))/SHORT_RATIO);
	}

	/**
	 * Computes the estimated distance to a target using the width of the particle in the image. For more information and graphics
	 * showing the math behind this approach see the Vision Processing section of the ScreenStepsLive documentation.
	 *
	 * @param image The image to use for measuring the particle estimated rectangle
	 * @param report The Particle Analysis Report for the particle
	 * @param isLong Boolean indicating if the target is believed to be the long side of a tote
	 * @return The estimated distance to the target in feet.
	 */
	double computeDistance (Image image, ParticleReport report, boolean isLong) {
		double normalizedWidth, targetWidth;
		NIVision.GetImageSizeResult size;

		size = NIVision.imaqGetImageSize(image);
		normalizedWidth = 2*(report.BoundingRectRight - report.BoundingRectLeft)/size.width;
		targetWidth = isLong ? 26.0 : 16.9;

		return  targetWidth/(normalizedWidth*12*Math.tan(VIEW_ANGLE*Math.PI/(180*2)));
	}
}
