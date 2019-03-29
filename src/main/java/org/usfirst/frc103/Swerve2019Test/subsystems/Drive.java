package org.usfirst.frc103.Swerve2019Test.subsystems;

import org.usfirst.frc103.Swerve2019Test.RobotMap;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc103.Swerve2019Test.RobotMap;

import java.util.Arrays;
import java.util.Collections;

import org.usfirst.frc103.Swerve2019Test.subsystems.Drive;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;


import org.usfirst.frc103.Swerve2019Test.commands.FieldCentricSwerveDrive;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import com.revrobotics.CANPIDController;

import edu.wpi.first.wpilibj.command.Subsystem;

public class Drive extends Subsystem {

	public TalonSRX steerLeftFront;
    public TalonSRX steerLeftRear;
    public TalonSRX steerRightFront;
    public TalonSRX steerRightRear;

    public CANSparkMax driveLeftFrontSpark;
    public CANSparkMax driveLeftRearSpark;
    public CANSparkMax driveRightFrontSpark;
    public CANSparkMax driveRightRearSpark;     
    
    // public CANPIDController driveLeftFrontController;
    // public CANPIDController driveLeftRearController;
    // public CANPIDController driveRightFrontController;
    // public CANPIDController driveRightRearController;
	public double WHEEL_BASE_LENGTH = 28.0;
	public double WHEEL_BASE_WIDTH = 22.0;
	public double ENCODER_COUNT_PER_ROTATION = 4098.0;

	//XXX: FIX THIS BEFORE COMPETITION, WHEEL DIAMETER SHOULD BE 4" maybe was 3.95
	public static final double WHEEL_DIAMETER = 4;
	public static final double MAX_SPEED = 0.3; //Max speed is 0 to 1 
	public static final double STEER_DEGREES_PER_COUNT = 360.0 / 4098.0, DRIVE_INCHES_PER_COUNT = (WHEEL_DIAMETER * Math.PI) / (80.0 * 6.67);
	public static final double DEADZONE = 0.08;
	public static final double MAX_REVERSIBLE_SPEED_DIFFERENCE = 0.5 * MAX_SPEED;
	private static final double DRIVE_P = 7.5, DRIVE_I = 0.0, DRIVE_D = 75.0, DRIVE_F = 1.7, DRIVE_RAMP_RATE = 0.2;
    private static final int DRIVE_I_ZONE = 0, DRIVE_ALLOWABLE_ERROR = 0, DRIVE_MEASUREMENT_WINDOW = 1;
    private static final VelocityMeasPeriod DRIVE_MEASUREMENT_PERIOD = VelocityMeasPeriod.Period_20Ms;
    private static final double STEER_P = 0.80, STEER_I = 0.0, STEER_D = 8.0;
    private static final int STATUS_FRAME_PERIOD = 5;
    private static final double RAMP_RATE = 0.5;

    
	
	public Drive(){

        driveLeftFrontSpark = new CANSparkMax(10, MotorType.kBrushless);
        driveLeftFrontSpark.restoreFactoryDefaults();
        driveLeftFrontSpark.setInverted(false);
        driveLeftFrontSpark.setOpenLoopRampRate(RAMP_RATE);
        driveLeftFrontSpark.setIdleMode(IdleMode.kBrake);
        // driveLeftFrontController = new CANPIDController(driveLeftFrontSpark);
        // driveLeftFrontController.setP(DRIVE_P);
        // driveLeftFrontController.setI(DRIVE_I);
        // driveLeftFrontController.setD(DRIVE_D);
        // driveLeftFrontController.setIZone(DRIVE_I_ZONE);
        // driveLeftFrontController.setFF(DRIVE_F);
        
        driveLeftRearSpark = new CANSparkMax(11, MotorType.kBrushless);
        driveLeftRearSpark.restoreFactoryDefaults();
        driveLeftRearSpark.setInverted(false);
        driveLeftRearSpark.setOpenLoopRampRate(RAMP_RATE);
        driveLeftRearSpark.setIdleMode(IdleMode.kBrake);
        // driveLeftRearController = new CANPIDController(driveLeftRearSpark);
        // driveLeftRearController.setP(DRIVE_P);
        // driveLeftRearController.setI(DRIVE_I);
        // driveLeftRearController.setD(DRIVE_D);
        // driveLeftRearController.setIZone(DRIVE_I_ZONE);
        // driveLeftRearController.setFF(DRIVE_F);

        driveRightFrontSpark = new CANSparkMax(12, MotorType.kBrushless);
        driveRightFrontSpark.restoreFactoryDefaults();
        driveRightFrontSpark.setInverted(false);
        driveRightFrontSpark.setOpenLoopRampRate(RAMP_RATE);
        driveRightFrontSpark.setIdleMode(IdleMode.kBrake);
        // driveRightFrontController = new CANPIDController(driveRightFrontSpark);
        // driveRightFrontController.setP(DRIVE_P);
        // driveRightFrontController.setI(DRIVE_I);
        // driveRightFrontController.setD(DRIVE_D);
        // driveRightFrontController.setIZone(DRIVE_I_ZONE);
        // driveRightFrontController.setFF(DRIVE_F);

        driveRightRearSpark = new CANSparkMax(13, MotorType.kBrushless);
        driveRightRearSpark.restoreFactoryDefaults();
        driveRightRearSpark.setInverted(false);
        driveRightRearSpark.setOpenLoopRampRate(RAMP_RATE);
        driveRightRearSpark.setIdleMode(IdleMode.kBrake);
        // driveRightRearController = new CANPIDController(driveRightRearSpark);
        // driveRightRearController.setP(DRIVE_P);
        // driveRightRearController.setI(DRIVE_I);
        // driveRightRearController.setD(DRIVE_D);
        // driveRightRearController.setIZone(DRIVE_I_ZONE);
		// driveRightRearController.setFF(DRIVE_F);
		
        steerLeftFront = new TalonSRX(16);
        steerLeftFront.configFactoryDefault();
        steerLeftFront.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 0);
        steerLeftFront.setInverted(true);
        steerLeftFront.config_kP(0, STEER_P, 0);
        steerLeftFront.config_kI(0, STEER_I, 0);
        steerLeftFront.config_kD(0, STEER_D, 0);
        steerLeftFront.config_IntegralZone(0, 100, 0);
        steerLeftFront.configAllowableClosedloopError(0, 5, 0);
        steerLeftFront.setNeutralMode(NeutralMode.Brake);
        steerLeftFront.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, STATUS_FRAME_PERIOD, 0);

        steerLeftRear = new TalonSRX(17);
        steerLeftRear.configFactoryDefault();
        steerLeftRear.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 0);
        steerLeftRear.setInverted(true);
        steerLeftRear.config_kP(0, STEER_P, 0);
        steerLeftRear.config_kI(0, STEER_I, 0);
        steerLeftRear.config_kD(0, STEER_D, 0);
        steerLeftRear.config_IntegralZone(0, 100, 0);
        steerLeftRear.configAllowableClosedloopError(0, 5, 0);
        steerLeftRear.setNeutralMode(NeutralMode.Brake);
        steerLeftRear.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, STATUS_FRAME_PERIOD, 0);

        steerRightFront = new TalonSRX(18);
        steerRightFront.configFactoryDefault();
        steerRightFront.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 0);
        steerRightFront.setInverted(true);
        steerRightFront.config_kP(0, STEER_P, 0);
        steerRightFront.config_kI(0, STEER_I, 0);
        steerRightFront.config_kD(0, STEER_D, 0);
        steerRightFront.config_IntegralZone(0, 100, 0);
        steerRightFront.configAllowableClosedloopError(0, 5, 0);
        steerRightFront.setNeutralMode(NeutralMode.Brake);
        steerRightFront.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, STATUS_FRAME_PERIOD, 0);

        steerRightRear = new TalonSRX(19);
        steerRightRear.configFactoryDefault();
        steerRightRear.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 0);
        steerRightRear.setInverted(true);
		steerRightRear.config_kP(0, STEER_P, 0);
        steerRightRear.config_kI(0, STEER_I, 0);
        steerRightRear.config_kD(0, STEER_D, 0);
        steerRightRear.config_IntegralZone(0, 100, 0);
        steerRightRear.configAllowableClosedloopError(0, 5, 0);
        steerRightRear.setNeutralMode(NeutralMode.Brake);
        steerRightRear.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, STATUS_FRAME_PERIOD, 0);

	}
	public void swerveDrive(double strafe, double forward, double omega) {
        double omegaL2 = omega * (WHEEL_BASE_LENGTH / 2.0);
        double omegaW2 = omega * (WHEEL_BASE_WIDTH / 2.0);
        
        // Compute the constants used later for calculating speeds and angles
        double A = strafe - omegaL2;
        double B = strafe + omegaL2;
        double C = forward - omegaW2;
        double D = forward + omegaW2;
        
        // Compute the drive motor speeds
        double speedLF = speed(B, D);
        double speedLR = speed(A, D);
        double speedRF = speed(B, C);
        double speedRR = speed(A, C);
        
        // ... and angles for the steering motors
    	double angleLF = angle(B, D) - 90;
    	double angleLR = angle(A, D) + 90;
    	double angleRF = angle(B, C) - 90;
    	double angleRR = angle(A, C) + 90;
    	
    	// Compute the maximum speed so that we can scale all the speeds to the range [0, 1]
    	double maxSpeed = Collections.max(Arrays.asList(speedLF, speedLR, speedRF, speedRR, 1.0));

    	// Set each swerve module, scaling the drive speeds by the maximum speed
    	setSwerveModule(steerLeftFront, driveLeftFrontSpark, angleLF, speedLF / maxSpeed);
    	setSwerveModule(steerLeftRear, driveLeftRearSpark, angleLR, speedLR / maxSpeed);
    	setSwerveModule(steerRightFront, driveRightFrontSpark, angleRF, speedRF / maxSpeed);
    	setSwerveModule(steerRightRear, driveRightRearSpark, angleRR, speedRR / maxSpeed);
	}
	
	private double speed(double val1, double val2){
    	return Math.hypot(val1, val2);
    }
    
    private double angle(double val1, double val2){
    	return Math.toDegrees(Math.atan2(val1, val2));
    }
	
	/*
	private void setSwerveModule(TalonSRX steer, TalonSRX drive, double angle, double speed) {
		// Get the current angle and speed for the module
		double currentAngle = steer.getSelectedSensorPosition(0) * STEER_DEGREES_PER_COUNT;
		double currentSpeed = drive.getSelectedSensorVelocity(0) * 10.0 * DRIVE_INCHES_PER_COUNT;

		SmartDashboard.putNumber("CurrentAngle", currentAngle);
		SmartDashboard.putNumber("Current Speed", currentSpeed);
		SmartDashboard.putNumber("Speed", speed);
		SmartDashboard.putNumber("Drive Velocity", drive.getSelectedSensorVelocity(0));
		SmartDashboard.putNumber("Drive Inch per Count", DRIVE_INCHES_PER_COUNT);

		// Calculate the number of degrees to turn assuming that speed is not reversed
		double angleDelta = Math.IEEEremainder(angle - currentAngle, 360.0);
		// Calculate the corresponding change in speed required
		double speedDifference = Math.abs(speed - currentSpeed);

		// Calculate the angle that requires the least amount of turning by allowing speed reversal
		double shortestAngleDelta = Math.IEEEremainder(angleDelta, 180.0);
		// If the previous calculation flipped the direction to turn, then the speed must be reversed
		double shortestSpeed = Math.signum(angleDelta) * Math.signum(shortestAngleDelta) * speed;
		// Calculate the change in speed when speed reversal is allowed
		double shortestSpeedDifference = Math.abs(shortestSpeed - currentSpeed);

		// If the change in speed required when using the angle that requires the least amount of turning is below the
		// speed reversal threshold, then always use that angle and corresponding speed
		// If the change in speed is above the reversal speed threshold, prefer the turning direction that results in
		// the least change in speed
		if (shortestSpeedDifference <= MAX_REVERSIBLE_SPEED_DIFFERENCE || shortestSpeedDifference <= speedDifference) {
			angleDelta = shortestAngleDelta;
			speed = shortestSpeed;
		}

		// Set the steering motor position and drive motor output accordingly
		steer.set(ControlMode.Position, (currentAngle + angleDelta) / STEER_DEGREES_PER_COUNT);
		drive.set(ControlMode.PercentOutput, speed / MAX_SPEED);
	}

	*/	


	
	private void setSwerveModule(TalonSRX steer, CANSparkMax drive, double angle, double speed) {
    	double currentPosition = steer.getSelectedSensorPosition(0);
    	double currentAngle = (currentPosition * 360.0 / ENCODER_COUNT_PER_ROTATION) % 360.0;
    	// The angle from the encoder is in the range [0, 360], but the swerve computations
    	// return angles in the range [-180, 180], so transform the encoder angle to this range
    	if (currentAngle > 180.0) {
    		currentAngle -= 360.0;
    	}
    	// TODO: Properly invert the steering motors so this isn't necessary
    	// This is because the steering encoders are inverted
    	double targetAngle = -angle;
    	double deltaDegrees = targetAngle - currentAngle;
    	// If we need to turn more than 180 degrees, it's faster to turn in the opposite direction
    	if (Math.abs(deltaDegrees) > 180.0) {
    		deltaDegrees -= 360.0 * Math.signum(deltaDegrees);
    	}
    	// If we need to turn more than 90 degrees, we can reverse the wheel direction instead and
		// only rotate by the complement
		
		//if (Math.abs(speed) <= MAX_SPEED){
    		if (Math.abs(deltaDegrees) > 90.0) {
    			deltaDegrees -= 180.0 * Math.signum(deltaDegrees);
    			speed = -speed;
			}
		//}
		

		double targetPosition = currentPosition + deltaDegrees * ENCODER_COUNT_PER_ROTATION / 360.0;
		steer.set(ControlMode.Position, targetPosition);
		drive.set(speed);

	}
	public void initDefaultCommand() {
		setDefaultCommand(new FieldCentricSwerveDrive());
    }
    
    public void encoderReset() {
//XXX actually fix this

//		RobotMap.driveLeftFrontSpark.setSelectedSensorPosition(0, 0, 0);
//    	RobotMap.driveRightFrontSpark.setSelectedSensorPosition(0, 0, 0);
//    	RobotMap.driveLeftFrontSpark.setSelectedSensorPosition(0, 0, 0);
//    	RobotMap.driveRightFrontSpark.setSelectedSensorPosition(0, 0, 0);
    }
	
}
