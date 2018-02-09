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
public class autotest1 {
	private double TacosFinancialSituation = 0.15;
	private TalonSRX flMotor = new TalonSRX(0);
	private TalonSRX blMotor = new TalonSRX(1);
	private TalonSRX frMotor = new TalonSRX(2);
	private TalonSRX brMotor = new TalonSRX(3);
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
	protected double limit(double value) {
	    if (value > 1.0) {
	      return 1.0;
	    }
	    if (value < -1.0) {
	      return -1.0;
	    }
	    return value;
	  }
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
	public autotest1() {
		// TODO Auto-generated method stub
		double x,y;
		x = -0.5;
		y= 0.5;
		arcadeDrive(x, y,false);
	}

}
