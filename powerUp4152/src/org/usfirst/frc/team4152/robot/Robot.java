/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package org.usfirst.frc.team4152.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.can.CANStatus;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
//import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.hal.HAL;
import edu.wpi.first.wpilibj.hal.FRCNetComm.tInstances;
import edu.wpi.first.wpilibj.hal.FRCNetComm.tResourceType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedControllerGroup;

import java.util.Random;

import javax.swing.text.DefaultStyledDocument.ElementBuffer;

import org.opencv.core.MatOfDMatch;

import com.ctre.CANTalon;
//import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifierJNI;
import com.ctre.phoenix.CANifierStatusFrame;
import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.IMotorControllerEnhanced;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
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
	SendableChooser autoChooser;
	private Joystick m_stick = new Joystick(0);
	ADXRS450_Gyro gyro = new ADXRS450_Gyro();
	private Encoder leftEncoder = new Encoder(1, 2);
	private Encoder rightEncoder = new Encoder(4, 5);
	// private Encoder leftClimbEncoder = new Encoder(5,6);
	// private Encoder rightClimbEncoder = new Encoder(7,8);
	private Timer m_timer = new Timer();
	private Spark lm = new Spark(2);
	private Spark rm = new Spark(0);
	char[] gameData;
	/*
	 * Don't do this, only trask does this.
	 * 
	 * private DifferentialDrive robotDrive = new DifferentialDrive(lm, rm);
	 */
	// drive motors, front to back, left to right
	private TalonSRX flMotor = new TalonSRX(0);
	private TalonSRX blMotor = new TalonSRX(1);
	private TalonSRX frMotor = new TalonSRX(2);
	private TalonSRX brMotor = new TalonSRX(3);
	private TalonSRX ClimbMotor1 = new TalonSRX(6);
	private TalonSRX ClimbMotor2 = new TalonSRX(7);
	// private Compressor compressor = new Compressor(0);
	// private Solenoid Ratchet = new Solenoid(3);
	 private Solenoid Arms = new Solenoid(4);
	 //private Solenoid Arms2 = new7
	final int buttonA = 1;
	final int buttonB = 2;
	final int buttonX = 3;
	final int buttonY = 4;
	final int LBumper = 5;
	final int RBumper = 6;
	final int buttonBack = 7;
	final int buttonStart = 8;
	final int lsPush = 9;
	final int rsPush = 10;
	final int rTrigger = 3;
	final int lTrigger = 4;
	private double TacosFinancialSituation = 0.15;
	int pulseWidthPos = 0;
	double ElevatorSpeed = 0;
	double ElevatorMaxSpeed = 0.75;
	double IntakeSpeed = 0.5;
	double wheelCircumference = 18.84;
	double EncoderPos = Math.abs(ClimbMotor2.getSelectedSensorPosition(0)) / 360;
	
	enum direction { left, right, up, down, out,none};

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override

	public void robotInit() {
		rightEncoder.setDistancePerPulse(wheelCircumference / 256);
		leftEncoder.setDistancePerPulse(wheelCircumference / 256);
		//compressor.setClosedLoopControl(true);
		ClimbMotor2.setSelectedSensorPosition(0, 0,0);
	}

	/**
	 * This function is run once each time the robot enters autonomous mode.
	 */
	@Override
	public void autonomousInit() {
		rightEncoder.reset();
		leftEncoder.reset();
		Arms.set(false);
		m_timer.start();
		hasntRun = true;
		//check in IDE as it may be incorrect in context.
		Hardware.Talon.setNeutralMode(com.ctre.phoenix.motorcontrol.NeutralMode.Brake);
		
	}
	boolean hasntRun = true;
	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		gameData = DriverStation.getInstance().getGameSpecificMessage().toCharArray();
		/*if (m_timer.get()<2) {
			arcadeDrive(0.7, 0);
		} else {
			arcadeDrive(0, 0);
		}*/
		if(hasntRun)
		{
			moveElevator(direction.up, 1.4, 0.5);
			hasntRun = false;
			testAuto();
			//moveElevator(direction.up, 1);
		}
	}
	
	
	
	public void testAuto(){
		System.out.println("in testAuto");
		//drive(0.7,18);
		//Timer.delay(2);
		drive(0.6, 96);
		if(gameData[0] =='L')
		{
			spitCube(direction.out);
		}
		//turnLeft(90);
		//turnRight(90);
	}

	/**
	 * This function is called once each time the robot enters teleoperated
	 * mode.
	 */
	@Override
	public void teleopInit() {
		//ClimbMotor2.setSelectedSensorPosition(0, 0,0);
		Arms.set(false);
		//check in IDE as it may be incorrect in context.
		Hardware.Talon.setNeutralMode(com.ctre.phoenix.motorcontrol.NeutralMode.Coast);
	}

	/**
	 * This function is called periodically during teleoperated mode.
	 */
	@Override
	public void teleopPeriodic() {
		EncoderPos = (Double)(Math.abs(ClimbMotor2.getSelectedSensorPosition(0))/4096.0);
		elevatorHandler();
		intakeHandler();
		armHandler();
		if(m_stick.getPOV() == 90)
		{
			rotate(direction.right);
		}
		if(m_stick.getPOV() == 270)
		{
			rotate(direction.left);
		}
		arcadeDrive(-m_stick.getRawAxis(1) * 0.85, m_stick.getRawAxis(0) * 0.85);
		SmartDashboard.putNumber("LeftEncoder", leftEncoder.getDistance());
		SmartDashboard.putNumber("RightEncoder", rightEncoder.getDistance());
		SmartDashboard.putNumber("LeftEncoder Rate", leftEncoder.getRate());
		SmartDashboard.putNumber("RightEncoder Rate", rightEncoder.getRate());
		SmartDashboard.putNumber("Elevator Encoder Value", EncoderPos);


	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}

	protected void intakeHandler()
	{
		//System.out.println("in intakeHandler");
		//intake
		if(m_stick.getRawButton(buttonA))
		{
			lm.set(-1);
			rm.set(1);
		}
		//outtake
		else if(m_stick.getRawButton(buttonX))
		{
			lm.set(1);
			rm.set(-1);
	
		}
		else if(m_stick.getPOV() == 90)
		{
			rotate(direction.left);
		}
		else if(m_stick.getPOV() == 270)
		{
			rotate(direction.right);
		}
		//stop motors
		else
		{
			lm.set(0);
			rm.set(0);
		}
	}
	
	protected void elevatorHandler() {
		//System.out.println("in elevatorHandler");
		if (m_stick.getRawButton(LBumper)) {
			// engaged = true;
			// Ratchet.set(!engaged);
			//elevator down
			if (EncoderPos > 0) {
				if (EncoderPos >= 1.5) {
					ElevatorSpeed = -ElevatorMaxSpeed;
				} else if (EncoderPos < 1.5) {
						ElevatorSpeed = -0.3;
					}
				}

			} else if (m_stick.getRawButton(RBumper)) {
				//engaged = false;
				//Ratchet.set(!engaged);
				//elevator up
				if (EncoderPos<8.6)
				{
					if (EncoderPos<=7)
					{
						ElevatorSpeed = ElevatorMaxSpeed;
					}
					else
					{ 
						if(EncoderPos>7)
						{
							ElevatorSpeed = 0.3;
						}
					}
				}
				else
				{
					ElevatorSpeed = 0;
				}

			} else {
				ElevatorSpeed = 0;
			}
		SmartDashboard.putNumber("Elevator Encoder value", EncoderPos);
		ClimbMotor1.set(ControlMode.PercentOutput, ElevatorSpeed);
		ClimbMotor2.set(ControlMode.PercentOutput, -ElevatorSpeed);

		}

	protected void armHandler()
	{
		if(m_stick.getRawButton(buttonY)&&m_stick.getRawButton(buttonB))
		{
			//raise piston
			//System.out.println("arms dropping v2");
			Arms.set(true);
		}
	}
	//I couldn't bring myself to delete this :(
	@Deprecated
	/**
	 * custom drive class for CAN bus.
	 * 
	 * @param motor1
	 *            front left motor
	 * @param mb
	 *            back left motor
	 * @param PowertrainSandwich
	 *            front right motor
	 * @param Cylinder00000100
	 *            back right motor
	 */
	private void TacoDrive(TalonSRX motor1, TalonSRX mb, TalonSRX PowertrainSandwich, TalonSRX Cylinder00000100) {
		if (Math.abs(m_stick.getRawAxis(1)) > TacosFinancialSituation) {
			if (m_stick.getRawAxis(0) > 0) {
				motor1.set(ControlMode.PercentOutput, -m_stick.getRawAxis(1));
				// mb.set(ControlMode.PercentOutput,- m_stick.getRawAxis(1));
				PowertrainSandwich.set(ControlMode.PercentOutput, -m_stick.getRawAxis(1) - (m_stick.getRawAxis(0) / 2));
				// Cylinder00000100.set(ControlMode.PercentOutput,
				// m_stick.getRawAxis(1)-(m_stick.getRawAxis(0)/2));
			} else {
				motor1.set(ControlMode.PercentOutput, -(m_stick.getRawAxis(1) + (m_stick.getRawAxis(0) / 2)));
				// may need to be reversed
				// mb.set(ControlMode.PercentOutput,-(
				// m_stick.getRawAxis(1)+(m_stick.getRawAxis(0)/2)));
				// motorL = y+x/2
				// motorRight = y
				PowertrainSandwich.set(ControlMode.PercentOutput, m_stick.getRawAxis(1));
				// Cylinder00000100.set(ControlMode.PercentOutput,
				// m_stick.getRawAxis(1));
			}
		} else if (Math.abs(m_stick.getRawAxis(0)) > TacosFinancialSituation) {
			// motorL = x, motorR = -x;
			motor1.set(ControlMode.PercentOutput, m_stick.getRawAxis(0));
			// mb.set(ControlMode.PercentOutput, m_stick.getRawAxis(0));
			PowertrainSandwich.set(ControlMode.PercentOutput, m_stick.getRawAxis(0));
			// Cylinder00000100.set(ControlMode.PercentOutput,
			// m_stick.getRawAxis(0));
		} else {
			motor1.set(ControlMode.PercentOutput, 0);
			// .set(ControlMode.PercentOutput,0);
			PowertrainSandwich.set(ControlMode.PercentOutput, 0);
			// Cylinder00000100.set(ControlMode.PercentOutput, 0);
		}
	}
	
	/**
	 * Arcade drive method for differential drive platform. The calculated
	 * values will be squared to decrease sensitivity at low speeds.
	 *
	 * @param xSpeed
	 *            The robot's speed along the X axis [-1.0..1.0]. Forward is
	 *            positive.
	 * @param zRotation
	 *            The robot's rotation rate around the Z axis [-1.0..1.0].
	 *            Clockwise is positive.
	 */
	@SuppressWarnings("ParameterName")
	public void arcadeDrive(double xSpeed, double zRotation) {
		arcadeDrive(xSpeed, zRotation, true);
	}

	/**
	 * Arcade drive method for differential drive platform.
	 *
	 * @param xSpeed
	 *            The robot's speed along the X axis [-1.0..1.0]. Forward is
	 *            positive.
	 *            Nolan is emperor of the Code Goblins. Sshhh...
	 * @param zRotation
	 *            The robot's rotation rate around the Z axis [-1.0..1.0].
	 *            Clockwise is positive.
	 * @param squaredInputs
	 *            If set, decreases the input sensitivity at low speeds.
	 */
	@SuppressWarnings("ParameterName")
	public void arcadeDrive(double xSpeed, double zRotation, boolean squaredInputs) {
		xSpeed = limit(xSpeed);
		xSpeed = applyDeadband(xSpeed, TacosFinancialSituation);
		int EncoderPos = Math.abs(ClimbMotor2.getSelectedSensorPosition(0)) / 360 / 100;

		zRotation = limit(zRotation);
		zRotation = applyDeadband(zRotation, TacosFinancialSituation);

		// Square the inputs (while preserving the sign) to increase fine
		// control
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
		// This Math Doesn't account for - direction values,ignore for now
		if (EncoderPos > 0)
			;
		{
			leftMotorOutput = limit(leftMotorOutput * (1.25 - EncoderPos / 100));
			rightMotorOutput = limit(rightMotorOutput * (1.25 - EncoderPos / 100));
		}
		flMotor.set(ControlMode.PercentOutput, limit(leftMotorOutput));
		blMotor.set(ControlMode.PercentOutput, limit(leftMotorOutput));
		brMotor.set(ControlMode.PercentOutput, -limit(rightMotorOutput));
		frMotor.set(ControlMode.PercentOutput, -limit(rightMotorOutput));

		// m_safetyHelper.feed();
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

	protected void drive(double speed, double inches) {
		leftEncoder.reset();
		rightEncoder.reset();
		double distanceLeft = 0.1;
		double distanceRight = 0.1;
		while ( distanceLeft < inches &&  distanceRight < inches) {
			if ( distanceLeft > distanceRight) {
				 arcadeDrive(speed, (-1.0+(distanceRight/distanceLeft)));
			} else {
				 arcadeDrive(speed, (1.0-(distanceLeft/distanceRight)));
			}
			distanceLeft = Math.abs(leftEncoder.getDistance());
			distanceRight = Math.abs(rightEncoder.getDistance());
			SmartDashboard.putNumber("leftEncoder", distanceLeft);
			SmartDashboard.putNumber("RightEncoder", distanceRight);
		}
		//arcadeDrive(0, 0);
		stop(speed);
	}

	protected void turnLeft(double distance) {
		int gyroAngle = 0;
		gyro.reset();
		while (gyroAngle < distance) {
			arcadeDrive(0, -1 + (gyroAngle / distance / 2));
			gyroAngle = (int) Math.abs(gyro.getAngle());
		}
		arcadeDrive(0, 0);
	}

	protected void turnRight(double distance) {
		int gyroAngle = 0;
		gyro.reset();
		while (gyroAngle < distance) {
			arcadeDrive(0, 1 - (gyroAngle / distance / 2));
			gyroAngle = (int) Math.abs(gyro.getAngle());
		}
		arcadeDrive(0,0);
	}
	protected void stop(double speed)
	{
		/*while(false)
		{
			//SmartDashboard.putNumber("left Encoder rate", Math.abs(leftEncoder.getRate()));
			//SmartDashboard.putNumber("right Encoder rate", Math.abs(rightEncoder.getRate()));
			//arcadeDrive(limit(-speed*2), 0);
		}*/
		arcadeDrive(0, 0);
	}
	protected void rotate(direction d)
	{
		if(d == direction.left)
		{
		lm.set(0.6);
		rm.set(0.6);
		}
		else if(d == direction.right)
		{
			lm.set(-0.6);
			rm.set(-0.6);
		}
	}
	protected void moveElevator(direction d, double positionWanted, double speedMod)
	{
		EncoderPos = Math.abs(ClimbMotor2.getSelectedSensorPosition(0))/4096.0;
		double speed = limit(positionWanted - EncoderPos);
		
		if(d == direction.down)
		{
			while(EncoderPos>positionWanted)
			{
				ClimbMotor1.set(ControlMode.PercentOutput, speed*speedMod);
				ClimbMotor2.set(ControlMode.PercentOutput, -speed*speedMod);
				EncoderPos = Math.abs(ClimbMotor2.getSelectedSensorPosition(0))/4096.0;
			}
		}
		else if(d == direction.up)
		{
			while(EncoderPos<positionWanted)
			{
				ClimbMotor1.set(ControlMode.PercentOutput, speed*speedMod);
				ClimbMotor2.set(ControlMode.PercentOutput, -speed*speedMod);
				EncoderPos = Math.abs(ClimbMotor2.getSelectedSensorPosition(0))/4096.0;
			}
		}
		ClimbMotor1.set(ControlMode.PercentOutput, 0);
		ClimbMotor2.set(ControlMode.PercentOutput, 0);
	}
	protected void spitCube(direction d)
	{
		if(d == direction.out)
		{
			lm.set(1);
			rm.set(-1);
		}
	}
}
