/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package org.usfirst.frc.team4152.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.can.CANStatus;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.hal.HAL;
import edu.wpi.first.wpilibj.hal.FRCNetComm.tInstances;
import edu.wpi.first.wpilibj.hal.FRCNetComm.tResourceType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedControllerGroup;

import com.ctre.CANTalon;
import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifierJNI;
import com.ctre.phoenix.CANifierStatusFrame;
import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.IMotorControllerEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	Command autonomousCommand;
	SendableChooser autoChooser ;
	private Joystick m_stick = new Joystick(0);
	private double joystickx = m_stick.getRawAxis(0);
	private double joysticky = m_stick.getRawAxis(1);
	private double hoistick = m_stick.getRawAxis(4);
	private double hoystick = m_stick.getRawAxis(5);
	private Encoder leftEncoder = new Encoder(1,2);
	private Encoder rightEncoder = new Encoder(3,4);
	private Encoder leftClimbEncoder = new Encoder(5,6);
	private Encoder rightClimbEncoder = new Encoder(7,8);
	private Timer m_timer = new Timer();
	//protected VictorSP leftMotors = new VictorSP(0);
	//protected VictorSP rightMotors = new VictorSP(1);
	//private DifferentialDrive robotDrive
	//= new DifferentialDrive(leftMotors, rightMotors);
	//creates talonSRX object with the id of 1.
	private TalonSRX t = new TalonSRX(2);
	private Spark lm = new Spark(0);
	private Spark rm = new Spark(1);
	//SpeedControllerGroup m_right = new SpeedControllerGroup(rm, rm);
	//SpeedControllerGroup m_left = new SpeedControllerGroup(lm, lm);
	private DifferentialDrive robotDrive
	= new DifferentialDrive(lm, rm);
	//drive motors, front to back, left to right
	private TalonSRX flMotor = new TalonSRX(0);
	private TalonSRX blMotor = new TalonSRX(1);
	private TalonSRX frMotor = new TalonSRX(2);
	private TalonSRX brMotor = new TalonSRX(3);
	
	String autoSelect = "0";
	private double TacosFinancialSituation = 0.15;
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	
	public void robotInit() {
		//SmartDashboard.putString("debug", "ROBOTInit");
		autoChooser= new SendableChooser();
		//sets the motor speed to 100%
		//t.set(ControlMode.PercentOutput, 0);
		leftEncoder.setSamplesToAverage(5);
		rightEncoder.setSamplesToAverage(5);
		leftEncoder.setDistancePerPulse(1.0/360);
		rightEncoder.setDistancePerPulse(1.0/360);
		leftEncoder.reset();
		rightEncoder.reset();
		leftClimbEncoder.setSamplesToAverage(5);
		rightClimbEncoder.setSamplesToAverage(5);
		leftClimbEncoder.setDistancePerPulse(1.0/360);
		rightClimbEncoder.setDistancePerPulse(1.0/360);
		leftClimbEncoder.reset();
		rightClimbEncoder.reset();
		
		//Auto-select
		
		
		//autoChooser.addDefault("Testboi", new autotest1());
		//uncomment previous line for auto testing
		//autoChooser.addObject("TestNoah", new autotest2());
		
		//autotest1 autoVersion1 = new autotest1(1);
		autoChooser.addDefault("Auto1", autoSelect = "1");
		//autotest2 autoVersion2 = new autotest2(0);
		autoChooser.addObject("Auto2", autoSelect = "2");
		//autotest3 autoVersion3 = new autotest3(0);
		autoChooser.addObject("Auto3",autoSelect = "3");
		//autotest4 autoVersion4 = new autotest4(0);
		autoChooser.addObject("Auto4",autoSelect = "4");
		//autotest5 autoVersion5 = new autotest5(0);
		autoChooser.addObject("Auto5",autoSelect = "5");
		//autotest6 autoVersion6 = new autotest6(0);
		autoChooser.addObject("Auto6",autoSelect = "6");
		
		SmartDashboard.putData("Autochoices and despair", autoChooser);
		
		//SmartDashboard.putString("Autonomous mode Chooser",autoChooser.getSelected().toString());
		//autoVersion1.someVar == 1;
		//autoVersion2.someVar == 1;
		
		
		/*
	

		if (autoVersion1.someVar == 1)
		{
			autoSelect = 1;
			SmartDashboard.putNumber("xValue", autoSelect);
		}
		
		else if (autoVersion2.someVar ==1)
		{
		    autoSelect = 2;
		    SmartDashboard.putNumber("xValue", autoSelect);
		}
		else if (autoVersion3.someVar ==1)
		{
		    autoSelect = 3;
		}
		else if (autoVersion4.someVar ==1)
		{
		    autoSelect = 4;
		}
		else if (autoVersion5.someVar ==1)
		{
		    autoSelect = 5;
		}
		else if (autoVersion6.someVar == 1)//somehow runs every if statement even though all but the first (should) be false. Even runs when changed to Somevar != 1 because shutup...
		{
		    autoSelect = 6;
		    SmartDashboard.putNumber("xValue", autoSelect);
		}
		
		//SmartDashboard.putString("debug", "finished teleopInit");
		 
		 
		*/
		//SmartDashboard.putNumber("xValue", autoSelect);

		
		
	}
	
	

	/**
	 * This function is run once each time the robot enters autonomous mode.
	 */
	@Override
	public void autonomousInit() {
		autoSelect = autoChooser.getSelected().toString();
		SmartDashboard.putString("My Choice",autoChooser.getSelected().toString());
		SmartDashboard.putString("debug", "autoInitLine156");
		m_timer.reset();
		m_timer.start();
		SmartDashboard.putString("debug", "line157");
		//leftEncoder.reset();
		//rightEncoder.reset();
		//leftClimbEncoder.reset();
		//rightClimbEncoder.reset();
		
		SmartDashboard.putString("debug", "aiTimerStart");
	
		
		//autonomousCommand = (Command) autoChooser.getSelected(); //dont uncomment
		SmartDashboard.putString("debug", "line166");
		//autonomousCommand.start(); //don't uncomment
		SmartDashboard.putString("debug", "line168");
		SmartDashboard.putString("Auto Selected",autoSelect);
		SmartDashboard.putString("debug", "line171");
		
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		SmartDashboard.putString("debug", "Periodic");
		Scheduler.getInstance().run();
		//if (autoVersion1. == 1); this line has been commented out
		//we shall overthrow the fuhrer with blood and iron, for today is the day of days that all days after this day shall remember this day as a day before their day that was the day of days... also Nolan is bad and so am I...
		// Drive for 2 seconds
		/*if (m_timer.get() < 2.0) {
			m_robotDrive.arcadeDrive(0.5, 0.0); // drive forwards half speed
		} else {
			m_robotDrive.stopMotor(); // stop robot
		}*/
		SmartDashboard.putNumber("timer", m_timer.get());
		if (m_timer.get() < 1.0)
		{
			if (autoSelect == "1")// dont f****** touch any code in regards to auto selection NOLAN!!!
			{
				//code for auto 1
				arcadeDrive(-1,0);
			}
			else if (autoSelect == "2")
			{
				//code for auto 2
				arcadeDrive (-0.5,0);
			}
			else if (autoSelect == "3")
			{
				//code for auto 3
				//arcadeDrive (-1,-1);
			}
			else if (autoSelect == "4")
			{
				//code for auto 4
			}
			else if (autoSelect == "5")
			{
				//code for auto 5
			}
			else if (autoSelect == "6")
			{
				//code for auto 6
			}
			else 
			{
				arcadeDrive(1,1);
			}
		}
		else
		{
			arcadeDrive(0,0);
		}
		SmartDashboard.putNumber("LeftEncoder Distance", leftEncoder.getDistance());
		SmartDashboard.putNumber("RightEncoder Distance", rightEncoder.getDistance());
		SmartDashboard.putNumber("LeftEncoder Rate", leftEncoder.getRate());
		SmartDashboard.putNumber("RightEncoder Rate", rightEncoder.getRate());
		SmartDashboard.putNumber("LeftClimbEncoder Distance", leftClimbEncoder.getDistance());
		SmartDashboard.putNumber("RightClimbEncoder Distance", rightClimbEncoder.getDistance());
		SmartDashboard.putNumber("LeftClimbEncoder Rate", leftClimbEncoder.getRate());
		SmartDashboard.putNumber("RightClimbEncoder Rate", rightClimbEncoder.getRate());
	}

	/**
	 * This function is called once each time the robot enters teleoperated mode.
	 */
	@Override
	public void teleopInit() {
		SmartDashboard.putString("debug", "teleopInit");
		leftEncoder.reset();
		rightEncoder.reset();
		leftClimbEncoder.reset();
		rightClimbEncoder.reset();
	}

	/**
	 * This function is called periodically during teleoperated mode.
	 */
	@Override
	public void teleopPeriodic() {
		/*if( (m_stick.getY() >=-0.2 && m_stick.getY() <= 0.2) && (m_stick.getX() >=-0.2 && m_stick.getX() <= 0.2))
		{
			robotDrive.arcadeDrive(hoystick*0.5, hoistick*0.5);
		}	
		else
		{
			robotDrive.arcadeDrive(m_stick.getY(), m_stick.getX());
		}*/
		//sets the motor with id 1 to 50%
		//t.set(ControlMode.PercentOutput, 50);
		//TacoDrive(flMotor,blMotor,frMotor,brMotor);
		arcadeDrive(-m_stick.getRawAxis(1), m_stick.getRawAxis(0));
		joystickx = m_stick.getRawAxis(0);
		joysticky = m_stick.getRawAxis(1);
		SmartDashboard.putNumber("Y Axis", joysticky);
		SmartDashboard.putNumber("X Axis", joystickx);
		SmartDashboard.putNumber("LeftEncoder Distance", leftEncoder.getDistance());
		SmartDashboard.putNumber("RightEncoder Distance", rightEncoder.getDistance());
		SmartDashboard.putNumber("LeftEncoder Rate", leftEncoder.getRate());
		SmartDashboard.putNumber("RightEncoder Rate", rightEncoder.getRate());
		SmartDashboard.putNumber("LeftClimbEncoder Distance", leftClimbEncoder.getDistance());
		SmartDashboard.putNumber("RightClimbEncoder Distance", rightClimbEncoder.getDistance());
		SmartDashboard.putNumber("LeftClimbEncoder Rate", leftClimbEncoder.getRate());
		SmartDashboard.putNumber("RightClimbEncoder Rate", rightClimbEncoder.getRate());
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}
	
	@Deprecated
	/**
	 * custom drive class for CAN bus.
	 * @param motor1 front left motor
	 * @param mb	back left motor
	 * @param PowertrainSandwich front right motor
	 * @param Cylinder00000100 back right motor
	 */
	private void TacoDrive(TalonSRX motor1, TalonSRX mb, TalonSRX PowertrainSandwich, TalonSRX Cylinder00000100 )
	{
		if(Math.abs(m_stick.getRawAxis(1)) > TacosFinancialSituation)
		{
			if(m_stick.getRawAxis(0)>0)
			{
				motor1.set(ControlMode.PercentOutput, -m_stick.getRawAxis(1));
				//mb.set(ControlMode.PercentOutput,- m_stick.getRawAxis(1));
				PowertrainSandwich.set(ControlMode.PercentOutput, -m_stick.getRawAxis(1)-(m_stick.getRawAxis(0)/2));
				//Cylinder00000100.set(ControlMode.PercentOutput, m_stick.getRawAxis(1)-(m_stick.getRawAxis(0)/2));
			}
			else
			{
				motor1.set(ControlMode.PercentOutput,-( m_stick.getRawAxis(1)+(m_stick.getRawAxis(0)/2)));
				//may need to be reversed
				//mb.set(ControlMode.PercentOutput,-( m_stick.getRawAxis(1)+(m_stick.getRawAxis(0)/2)));
				//motorL = y+x/2
				//motorRight = y
				PowertrainSandwich.set(ControlMode.PercentOutput, m_stick.getRawAxis(1));
				//Cylinder00000100.set(ControlMode.PercentOutput, m_stick.getRawAxis(1));
			}
		}
		else if(Math.abs(m_stick.getRawAxis(0)) > TacosFinancialSituation)
		{
			// motorL = x, motorR = -x;
			motor1.set(ControlMode.PercentOutput, m_stick.getRawAxis(0));
			//mb.set(ControlMode.PercentOutput, m_stick.getRawAxis(0));
			PowertrainSandwich.set(ControlMode.PercentOutput, m_stick.getRawAxis(0));
			//Cylinder00000100.set(ControlMode.PercentOutput, m_stick.getRawAxis(0));
		}
		else
		{
			motor1.set(ControlMode.PercentOutput, 0);
			//.set(ControlMode.PercentOutput,0);
			PowertrainSandwich.set(ControlMode.PercentOutput, 0);
			//Cylinder00000100.set(ControlMode.PercentOutput, 0);
		}
	}
	
	/**
	   * Arcade drive method for differential drive platform.
	   * The calculated values will be squared to decrease sensitivity at low speeds.
	   *
	   * @param xSpeed    The robot's speed along the X axis [-1.0..1.0]. Forward is positive.
	   * @param zRotation The robot's rotation rate around the Z axis [-1.0..1.0]. Clockwise is
	   *                  positive.
	   */
	  @SuppressWarnings("ParameterName")
	  public void arcadeDrive(double xSpeed, double zRotation) {
	    arcadeDrive(xSpeed, zRotation, true);
	  }

	  /**
	   * Arcade drive method for differential drive platform.
	   *
	   * @param xSpeed        The robot's speed along the X axis [-1.0..1.0]. Forward is positive.
	   * @param zRotation     The robot's rotation rate around the Z axis [-1.0..1.0]. Clockwise is
	   *                      positive.
	   * @param squaredInputs If set, decreases the input sensitivity at low speeds.
	   */
	  @SuppressWarnings("ParameterName")
	  public void arcadeDrive(double xSpeed, double zRotation, boolean squaredInputs) {
	    xSpeed = limit(xSpeed);
	    xSpeed = applyDeadband(xSpeed, TacosFinancialSituation);

	    zRotation = limit(zRotation);
	    zRotation = applyDeadband(zRotation, TacosFinancialSituation);

	    // Square the inputs (while preserving the sign) to increase fine control
	    // while permitting full power.
	    if (squaredInputs) {
	      xSpeed = Math.copySign(xSpeed * xSpeed, xSpeed);
	      zRotation = Math.copySign(zRotation * zRotation, zRotation);
	    }

	    double leftMotorOutput;
	    double rightMotorOutput;

	    double maxInput = Math.copySign(Math.max(Math.abs(xSpeed), Math.abs(zRotation)), xSpeed);

	    if (xSpeed >= 0.0) {
	      // First quadrant, else second quadrant
	      if (zRotation >= 0.0) {
	        leftMotorOutput = maxInput;
	        rightMotorOutput = xSpeed - zRotation;
	      } else {
	        leftMotorOutput = xSpeed + zRotation;
	        rightMotorOutput = maxInput;
	      }
	    } else {
	      // Third quadrant, else fourth quadrant
	      if (zRotation >= 0.0) {
	        leftMotorOutput = xSpeed + zRotation;
	        rightMotorOutput = maxInput;
	      } else {
	        leftMotorOutput = maxInput;
	        rightMotorOutput = xSpeed - zRotation;
	      }
	    }

	    flMotor.set(ControlMode.PercentOutput,limit(leftMotorOutput));
	    blMotor.set(ControlMode.PercentOutput,limit(leftMotorOutput));
	    brMotor.set(ControlMode.PercentOutput,-limit(rightMotorOutput));
	    frMotor.set(ControlMode.PercentOutput, -limit(rightMotorOutput));

	    //m_safetyHelper.feed();
	  }
	  /**
	   * Limit motor values to the -1.0 to +1.0 range.
	   */
	  protected double limit(double value) {
	    if (value > 1.0) {
	      return 1.0;
	    }
	    if (value < -1.0) {
	      return -1.0;
	    }
	    return value;
	  }
	@Deprecated
	private double capNumber(double capping)
	{
		if(Math.abs(capping)>1)
		{
			if(capping<0) return (double) -1;
			else return (double) 1;
		}
		return capping;
	}
	protected double applyDeadband(double value, double deadband) {
	    if (Math.abs(value) > deadband) {
	      if (value > 0.0) {
	        return (value - deadband) / (1.0 - deadband);
	      } else {
	        return (value + deadband) / (1.0 - deadband);
	      }
	    } else {
	      return 0.0;
	    }
	  }
}
