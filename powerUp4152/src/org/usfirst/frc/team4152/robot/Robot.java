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
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
//import edu.wpi.first.wpilibj.SpeedControllerGroup;
//import com.ctre.phoenix.motorcontrol.ControlMode;
//import com.ctre.phoenix.motorcontrol.can.TalonSRX;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	private Joystick m_stick = new Joystick(0);
	private double hoistick = m_stick.getRawAxis(4);
	private double hoystick = m_stick.getRawAxis(5);
	private Timer m_timer = new Timer();
	//protected VictorSP leftMotors = new VictorSP(0);
	//protected VictorSP rightMotors = new VictorSP(1);
	//private DifferentialDrive robotDrive
	//= new DifferentialDrive(leftMotors, rightMotors);
	//creates talonSRX object
	//private TalonSRX t = new TalonSRX(2);
	private Spark lm = new Spark(0);
	private Spark rm = new Spark(1);
	//SpeedControllerGroup m_right = new SpeedControllerGroup(rm, rm);
	//SpeedControllerGroup m_left = new SpeedControllerGroup(lm, lm);
	private DifferentialDrive robotDrive
	= new DifferentialDrive(lm, rm);

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		//sets the motor speed to 100%
		//t.set(ControlMode.PercentOutput, 0);
	}

	/**
	 * This function is run once each time the robot enters autonomous mode.
	 */
	@Override
	public void autonomousInit() {
		m_timer.reset();
		m_timer.start();
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		// Drive for 2 seconds
		/*if (m_timer.get() < 2.0) {
			m_robotDrive.arcadeDrive(0.5, 0.0); // drive forwards half speed
		} else {
			m_robotDrive.stopMotor(); // stop robot
		}*/
	}

	/**
	 * This function is called once each time the robot enters teleoperated mode.
	 */
	@Override
	public void teleopInit() {
	}

	/**
	 * This function is called periodically during teleoperated mode.
	 */
	@Override
	public void teleopPeriodic() {
		if( (m_stick.getY() >=-0.2 && m_stick.getY() <= 0.2) && (m_stick.getX() >=-0.2 && m_stick.getX() <= 0.2))
		{
			robotDrive.arcadeDrive(hoystick*0.5, hoistick*0.5);
		}	
		else
		{
			robotDrive.arcadeDrive(m_stick.getY(), m_stick.getX());
		}
			
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}
}
