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
import edu.wpi.first.wpilibj.Compressor;
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
	private Spark lm = new Spark(2);
	private Spark rm = new Spark(0);
	//SpeedControllerGroup m_right = new SpeedControllerGroup(rm, rm);
	//SpeedControllerGroup m_left = new SpeedControllerGroup(lm, lm);
	private DifferentialDrive robotDrive
	= new DifferentialDrive(lm, rm);
	//drive motors, front to back, left to right
	private TalonSRX flMotor = new TalonSRX(0);
	private TalonSRX blMotor = new TalonSRX(1);
	private TalonSRX frMotor = new TalonSRX(2);
	private TalonSRX brMotor = new TalonSRX(3);
	private TalonSRX ClimbMotor1 = new TalonSRX(6);
	private TalonSRX ClimbMotor2 = new TalonSRX(7);
	//private Compressor Compressor = new Compressor(5);
	/*private Solenoid Ratchet = new Solenoid (0);
	private Solenoid leftArms= new Solenoid (1);
	private Solenoid rightArms = new Solenoid (2);*/
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
	
	
	String autoSelect = "0";
	private double TacosFinancialSituation = 0.15;
	int pulseWidthPos = 0;
	double ElevatorSpeed = 0;
	double ElevatorMaxSpeed = 0.5;
	double IntakeSpeed = 0.5;
	
	
			
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	
	public void robotInit() {
		//PlotThread _plotThread;
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
		ClimbMotor1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 0);
		ClimbMotor2.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 0);
		
		//Compressor.setClosedLoopControl(true);

		autoChooser.addDefault("Auto1", autoSelect = "1");
		autoChooser.addObject("Auto2", autoSelect = "2");
		autoChooser.addObject("Auto3",autoSelect = "3");
		autoChooser.addObject("Auto4",autoSelect = "4");
		autoChooser.addObject("Auto5",autoSelect = "5");
		autoChooser.addObject("Auto6",autoSelect = "6");
		
		SmartDashboard.putData("Autochoices and despair", autoChooser);

	
	}
	
	
	public String gameData;
	/**
	 * This function is run once each time the robot enters autonomous mode.
	 */
	@Override
	public void autonomousInit() {
		//ClimbMotor1.set;
		leftEncoder.reset();
		rightEncoder.reset();
		leftClimbEncoder.reset();
		rightClimbEncoder.reset();
		ClimbMotor1.setSelectedSensorPosition(0, 0 ,0);
		ClimbMotor2.setSelectedSensorPosition(0, 0 ,0);
		gameData = DriverStation.getInstance().getGameSpecificMessage();
		autoSelect = autoChooser.getSelected().toString();
		SmartDashboard.putString("My Choice",autoChooser.getSelected().toString());
		m_timer.reset();
		m_timer.start();
		
		double getDist = leftEncoder.get() *18.84;
		{
			
		}
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		double EncoderPos = ClimbMotor2.getSelectedSensorPosition(0)/360;
		Scheduler.getInstance().run();
		SmartDashboard.putNumber("Elevator Encoder Velocity",ClimbMotor2.getSensorCollection().getPulseWidthVelocity());
		//we shall overthrow the fuhrer with blood and iron, for today is the day of days that all days after this day shall remember this day as a day before their day that was the day of days... also Nolan is bad and so am I...
		SmartDashboard.putNumber("timer", m_timer.get());
		/*
			while(EncoderPos<25)
			{
				ClimbMotor1.set(ControlMode.PercentOutput, 0.5);
				ClimbMotor2.set(ControlMode.PercentOutput, -0.5);
				lm.set(-1);
				rm.set(-1);
			}
			while(EncoderPos>0)
			{
				ClimbMotor1.set(ControlMode.PercentOutput, -0.5);
				ClimbMotor2.set(ControlMode.PercentOutput, 0.5);
				rm.set(0);
				lm.set(0);
			}
			*/
			if (autoSelect == "1")// dont touch any code in regards to auto selection, NOLAN!!!
			{
				if(gameData.length()>0)
				{
					if(gameData.charAt(0) == 'L')
					{
						//while (leftEncoder.getDistance()<2.0&& rightEncoder.getDistance()<2.0)
						while(m_timer.get()<2.0)
						{
							arcadeDrive(-0.5,0);
						}
						while(m_timer.get()<5.0)
						{	
							if(gameData.charAt(1)=='L')
							{
								arcadeDrive(-0.5,0.5);
							}
							else
							{
								arcadeDrive(-0.5,-0.5);
							}
						}
						arcadeDrive(0,0);
					} 
					
					else
					{
						//while (leftEncoder.getDistance()<2.0&& rightEncoder.getDistance()<2.0)
						while(m_timer.get()<2.0)
;						{
							arcadeDrive(0.5,0);
						}
						while(m_timer.get()<5.0)
						{
							if(gameData.charAt(1)=='L')
							{
								arcadeDrive(0.5,0.5);
							}
							else
							{
								arcadeDrive(0.5,-0.5);
							}
						}
					}
				}	
			}
			else if (autoSelect == "2")
			{
				
				//code for auto 2
				if (gameData.length()>1)
				{
					if (gameData.charAt(0) == 'L')
					{	
						while (leftEncoder.getDistance()<9&& rightEncoder.getDistance()<9) 
						{
							//Drive Forward
							arcadeDrive (0.25,0);
							/*while(EncoderPos<50)
							{
								//Raise Elevator
								ClimbMotor1.set(ControlMode.PercentOutput, 1);
								ClimbMotor2.set(ControlMode.PercentOutput, -1);
							}*/
							
						}
						while(leftEncoder.getDistance()<11)
						{
							//Turn Right 90deg
							arcadeDrive(0.25,-0.25);
						}
						arcadeDrive(0,0);
						//Kick Out Cube
						lm.set(-1);
						rm.set(-1);
						Timer.delay(0.3);
						/*while(EncoderPos>0)
						{
							//Lower Elevator
							ClimbMotor1.set(ControlMode.PercentOutput, -1);
							ClimbMotor2.set(ControlMode.PercentOutput, 1);
						}*/
						arcadeDrive(0,0);
						ClimbMotor1.set(ControlMode.PercentOutput, 0);
						ClimbMotor2.set(ControlMode.PercentOutput, 0);
					}
					else if (gameData.charAt(0) == 'R')
					{
						while(leftEncoder.getDistance()<10&&rightEncoder.getDistance()<10)
						{
							//Drive Forward
							arcadeDrive(1,0);
						}
						while(leftEncoder.getDistance()<12&&rightEncoder.getDistance()<10)
						{
							//Turn Right 90deg
							arcadeDrive(1,-1);
						}
						while(leftEncoder.getDistance()<20&&rightEncoder.getDistance()<18)
						{
							//Drive Forward
							arcadeDrive(1,0);
						}
						while(leftEncoder.getDistance()<22&&rightEncoder.getDistance()<18)
						{
							//Turn Right 90 deg
							arcadeDrive(1,-1);
							while(EncoderPos<50)
							{
								//Raise Elevator
								ClimbMotor1.set(ControlMode.PercentOutput, 1);
								ClimbMotor2.set(ControlMode.PercentOutput, -1);
							}
						}
						//Run Intake Out
						lm.set(-1);
						rm.set(-1);
						Timer.delay(0.3);
						while(EncoderPos>0)
						{
							//Lower Elevator
							ClimbMotor1.set(ControlMode.PercentOutput, -1);
							ClimbMotor2.set(ControlMode.PercentOutput, 1);
						}
						arcadeDrive(0,0);
						ClimbMotor1.set(ControlMode.PercentOutput, 0);
						ClimbMotor2.set(ControlMode.PercentOutput, 0);
					}
						
				}
			}
			else if (autoSelect == "3")
			{
				//code for auto 3
				if (gameData.length()>0)
				{
					if (gameData.charAt(0)=='R')
					{
						while(leftEncoder.get() < 9 && rightEncoder.get() < 9)
						{
							//Drive Forward
							arcadeDrive(-1,0);
						}
						while(rightEncoder.get()<11)
						{
							//Turn left 90deg
							arcadeDrive(-1,1);
							while(EncoderPos<50)
							{
								//Elevator Up
								ClimbMotor1.set(ControlMode.PercentOutput, 1);
								ClimbMotor2.set(ControlMode.PercentOutput, -1);
							}
							//Intake In
							lm.set(-1);
							rm.set(-1);
							Timer.delay(0.3);
							while(EncoderPos>0)
							{
								//Lower Elevator
								ClimbMotor1.set(ControlMode.PercentOutput, -1);
								ClimbMotor2.set(ControlMode.PercentOutput, 1);
							}
						}
						arcadeDrive(0,0);
						ClimbMotor1.set(ControlMode.PercentOutput, 0);
						ClimbMotor2.set(ControlMode.PercentOutput, 0);
					}
					else if (gameData.charAt(0)=='L')
					{
						while(leftEncoder.get()<10&&rightEncoder.get()<10)
						{
							//Drive Forward
							arcadeDrive(-1,0);
						}
						while(rightEncoder.get()<11)
						{
							//Turn left 90deg
							arcadeDrive(-1,1);
						}
						while(leftEncoder.get()<19&&rightEncoder.get()<18)
						{
							//Drive Forward
							arcadeDrive(-1,0);
						}
						while(rightEncoder.get()<20)
						{
							//Turn 90deg Left
							arcadeDrive(-1,1);
							while(EncoderPos<50)
							{
								//Elevator Up
								ClimbMotor1.set(ControlMode.PercentOutput, 1);
								ClimbMotor2.set(ControlMode.PercentOutput, -1);
							}
							lm.set(-1);
							rm.set(-1);
							Timer.delay(0.3);
							while(EncoderPos>0)
							{
								//Elevator Down
								ClimbMotor1.set(ControlMode.PercentOutput, -1);
								ClimbMotor2.set(ControlMode.PercentOutput, 1);
							}
						}
						arcadeDrive(0,0);
						ClimbMotor1.set(ControlMode.PercentOutput, 0);
						ClimbMotor2.set(ControlMode.PercentOutput, 0);
					}
				}
			}
			else if (autoSelect == "4")
			{
				//code for auto 4
			}
			else if (autoSelect == "5")
			{
				//code for auto 5
				while(m_timer.get()<1.0)
				{
					lm.set(-1);
					rm.set(1);
				}
				while((ClimbMotor2.getSelectedSensorPosition(0)/360)<98)
				//while(m_timer.get()<4.7)
				{	
					if(EncoderPos>87)
					{
						ElevatorSpeed = (EncoderPos/100) +0.25;
					}
					else 
					{
						ElevatorSpeed = 1;
					}
				ClimbMotor1.set(ControlMode.PercentOutput, ElevatorSpeed);
				ClimbMotor2.set(ControlMode.PercentOutput, -ElevatorSpeed);
				}
				if (EncoderPos==98)
				{
				lm.set(1);
				rm.set(-1);
				}
				//while(m_timer.get()<9.4)
				while((ClimbMotor2.getSelectedSensorPosition(0)/360)>0)
				{
					if (EncoderPos<10)
					{
						ElevatorSpeed = 0.25;
					}
					else
					{
						ElevatorSpeed = 1;
					}
				ClimbMotor1.set(ControlMode.PercentOutput, -ElevatorSpeed);
				ClimbMotor2.set(ControlMode.PercentOutput, ElevatorSpeed);
				}
				ClimbMotor1.set(ControlMode.PercentOutput, 0);
				ClimbMotor2.set(ControlMode.PercentOutput, 0);
				}
			else if (autoSelect == "6")
			{
				//code for auto 6
			}
		//}
		SmartDashboard.putNumber("LeftEncoder Distance", leftEncoder.getDistance());
		SmartDashboard.putNumber("RightEncoder Distance", rightEncoder.getDistance());
		SmartDashboard.putNumber("LeftEncoder Rate", leftEncoder.getRate());
		SmartDashboard.putNumber("RightEncoder Rate", rightEncoder.getRate());
	}

	/**
	 * This function is called once each time the robot enters teleoperated mode.
	 */
	@Override
	public void teleopInit() {
		leftEncoder.reset();
		rightEncoder.reset();
		leftClimbEncoder.reset();
		rightClimbEncoder.reset();
		ElevatorSpeed = 0;
		ClimbMotor1.setSelectedSensorPosition(0, 0 ,0);
		ClimbMotor2.setSelectedSensorPosition(0, 0 ,0);
	}

	/**
	 * This function is called periodically during teleoperated mode.
	 */
	@Override
	public void teleopPeriodic() {
		
		arcadeDrive(-m_stick.getRawAxis(1) * 0.75, m_stick.getRawAxis(0) * 0.75);
		joystickx = m_stick.getRawAxis(0);
		joysticky = m_stick.getRawAxis(1);
		SmartDashboard.putNumber("LeftEncoder Distance", leftEncoder.getDistance());
		SmartDashboard.putNumber("RightEncoder Distance", rightEncoder.getDistance());
		SmartDashboard.putNumber("LeftEncoder Rate", leftEncoder.getRate());
		SmartDashboard.putNumber("RightEncoder Rate", rightEncoder.getRate());
		int EncoderPos = ClimbMotor2.getSelectedSensorPosition(0)/360;
		SmartDashboard.putNumber("Elevator Encoder Value", EncoderPos);
		if(m_stick.getRawButton(RBumper))
		{
			if (EncoderPos<97)
			{
				if (EncoderPos<=87)
				{
					ElevatorSpeed = ElevatorMaxSpeed;
				}
				else
				{ 
					if(EncoderPos>87)
					{
						ElevatorSpeed = (EncoderPos/100) +0.25;
					}
				}
			}
			else
			{
				ElevatorSpeed = 0;
			}
			ClimbMotor1.set(ControlMode.PercentOutput, ElevatorSpeed);
			ClimbMotor2.set(ControlMode.PercentOutput, -ElevatorSpeed);
		  }
		else if (m_stick.getRawButton(LBumper))
		{
			if(EncoderPos>0)
			{
				if (EncoderPos>=10)
				{
					ElevatorSpeed = ElevatorMaxSpeed;
				}
			else
			{ 
				if (EncoderPos<15)
				{
					ElevatorSpeed = 0.15;
				}
			}
			}
			else
			{
				ElevatorSpeed = 0;
			}
			ClimbMotor1.set(ControlMode.PercentOutput, -ElevatorSpeed);
			ClimbMotor2.set(ControlMode.PercentOutput, ElevatorSpeed);
		
			
		}
		else
		{
			ClimbMotor1.set(ControlMode.PercentOutput, 0);
			ClimbMotor2.set(ControlMode.PercentOutput, 0);
		}
		SmartDashboard.putNumber("Elevator Speed", ElevatorSpeed);
		//Intake
		if (m_stick.getRawButton(buttonX))
		{
			lm.set(1);
			rm.set(-1);
		}
		//Outtake(phrasing)
		else if(m_stick.getRawButton(buttonA))
		{
			lm.set(-1);
			rm.set(1);
		}
		else
		{
		//No Trigger Input
			lm.set(0);
			rm.set(0);
		}
		/*if (m_timer.get()>145)
		{
			if (m_stick.getRawButton(buttonY))
			{
				leftArms.set(true);
				rightArms.set(true);
			}
			else if (m_stick.getRawButton(buttonB))
			{
				Ratchet.set(false);
			}
			else
			{
				leftArms.set(false);
				rightArms.set(false);
				Ratchet.set(true);
			}
		}*/
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
	    int EncoderPos = ClimbMotor2.getSelectedSensorPosition(0)/360/100;

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
	    //This Math Doesn't account for - direction values,ignore for now
	    if (EncoderPos>0);
	    {
	    	leftMotorOutput = limit(leftMotorOutput*(1.25-EncoderPos/100));
	    	rightMotorOutput = limit(rightMotorOutput*(1.25-EncoderPos/100));
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
