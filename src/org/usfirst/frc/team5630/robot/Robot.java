package org.usfirst.frc.team5630.robot;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot
{
	// Variable declarations
	final int AccelerationCoefficient = 1;
	final int duration = 20;
	final String defaultAuto = "Default";
	final String sideShoot = "Blue Shoot";
	final String middleGear = "Middle Gear";
	final String rightGear = "Right Gear";
	final String leftGear = "Left Gear";
	final double DeadZone = 0.215;
	int startEncoderTicks;
	String autoSelected;
	final int MaxRPM = 450;
	SendableChooser<String> chooser = new SendableChooser<>();
//	CANTalon shooter1, shooter2;
	CANTalon leftSRX1, rightSRX1, leftSRX2, rightSRX2;
	CANTalon leftSRX3, rightSRX3;
	CANTalon intake, index, arm;
	Joystick joystick1, joystick2;
	double rightX1, rightY1, leftTrigger1, rightTrigger1, leftX1, leftY1;
	boolean buttonA1, buttonB1, buttonX1, buttonY1, buttonRB1, buttonLB1, buttonLeftStickClick1, buttonRightStickClick1,
			buttonBack1, buttonStart1;
	boolean buttonALast1, buttonBLast1, buttonXLast1, buttonYLast1, buttonRBLast1, buttonLBLast1,
			buttonLeftStickClickLast1, buttonRightStickClickLast1, buttonBackLast1, buttonStartLast1;
	double rightX2, rightY2, leftTrigger2, rightTrigger2, leftX2, leftY2;
	boolean buttonA2, buttonB2, buttonX2, buttonY2, buttonRB2, buttonLB2, buttonLeftStickClick2, buttonRightStickClick2,
			buttonBack2, buttonStart2;
	int buttonDPad1, buttonDPad2;
	boolean buttonALast2, buttonBLast2, buttonXLast2, buttonYLast2, buttonRBLast2, buttonLBLast2,
			buttonLeftStickClickLast2, buttonRightStickClickLast2, buttonBackLast2, buttonStartLast2;
	int buttonDPadLast1, buttonDPadLast2;
	double shooterSpeed;
	boolean shooterToggle, intakeToggle;
	int numberOfPWMisZeroinARow;
	RobotDrive robotDrive;
	Timer timer = new Timer();
	
	AHRS navx = new AHRS(SPI.Port.kMXP);
	double forwardSpeed = 0.0;
	PIDController headingController = new PIDController(10.0, 0.0, 0.0, navx, new PIDOutput() {
		@Override
		public void pidWrite(double output) {
//			System.out.println("Driving heading " + headingController.getSetpoint() + " with speed "
//		+ forwardSpeed + " and output " + output 
//		+ ". current angle = " + navx.getAngle() + " and speed " + leftSRX1.getSpeed());
			leftSRX1.changeControlMode(TalonControlMode.Speed);
			rightSRX1.changeControlMode(TalonControlMode.Speed);
			leftSRX1.set(-(forwardSpeed + output));
			rightSRX1.set((forwardSpeed - output));	
		}
	});

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit()
	{
		chooser.addDefault(rightGear, rightGear);
		chooser.addObject(sideShoot, sideShoot);
		chooser.addObject(middleGear, middleGear);
		chooser.addObject(leftGear, leftGear);
		SmartDashboard.putData("Auto chooser", chooser);
		leftSRX1 = new CANTalon(1);
		leftSRX1.enableBrakeMode(true);
		leftSRX2 = new CANTalon(2);
		leftSRX2.enableBrakeMode(true);
		// leftSRX3 = new CANTalon(3);
		leftSRX3 = new CANTalon(5);
		leftSRX3.enableBrakeMode(true);
		arm = new CANTalon(4);
		arm.enableBrakeMode(true);
		rightSRX3 = new CANTalon(6);
		rightSRX3.enableBrakeMode(true);
		rightSRX1 = new CANTalon(8);
		rightSRX1.enableBrakeMode(true);
		rightSRX2 = new CANTalon(7);
		rightSRX2.enableBrakeMode(true);
		// rightSRX3 = new CANTalon(9);
		intake = new CANTalon(9);
		intake.enableBrakeMode(false);
		index = new CANTalon(3);
		index.enableBrakeMode(false);
		joystick1 = new Joystick(0);
		joystick2 = new Joystick(1);
		// intakeMotor.setInverted(true);
		arm.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
//		arm.configEncoderCodesPerRev(4096);
		// // shooter1.setFeedbackDevice(FeedbackDevice.QuadEncoder);
		// // shooter1.configEncoderCodesPerRev(4096);
		leftSRX1.setFeedbackDevice(FeedbackDevice.QuadEncoder);
		leftSRX1.configEncoderCodesPerRev(180);
		leftSRX1.enableBrakeMode(true);
		leftSRX2.enableBrakeMode(true);
		leftSRX3.enableBrakeMode(true);
		// System.out.println("F is: " + leftSRX1.getF());
		leftSRX1.setPID(2, 0.0011, 0);
		leftSRX1.reverseSensor(true);
		leftSRX1.setInverted(true);
		leftSRX1.setInverted(true);
		leftSRX2.setInverted(true);
		leftSRX3.reverseOutput(false);
		leftSRX2.reverseOutput(false);
		leftSRX3.reverseOutput(false);
		rightSRX1.setFeedbackDevice(FeedbackDevice.QuadEncoder);
		rightSRX1.configEncoderCodesPerRev(180);
		rightSRX1.setPID(2, 0.0011, 0);
		rightSRX1.reverseSensor(false);
		rightSRX1.setInverted(false);
		rightSRX2.setInverted(false);
		rightSRX3.setInverted(false);
		rightSRX1.reverseOutput(true);
		rightSRX2.reverseOutput(false);
		rightSRX3.reverseOutput(false);
		arm.setInverted(true);
		arm.reverseSensor(false);
		arm.configEncoderCodesPerRev(4096);
		arm.setReverseSoftLimit(0);
		arm.setForwardSoftLimit(0.125);
		arm.setPID(0.1, 0.0001, 0);
		arm.setF(100);
		intake.setInverted(false);
		index.setInverted(true);

//		// shooter1.setInverted(false);
//		shooter2.setInverted(false);
//		// shooter1.reverseOutput(true);
//		// shooter1.reverseSensor(true);
		arm.configPeakOutputVoltage(4, -4);
		rightSRX1.configPeakOutputVoltage(12.0f, -12.0f);
		rightSRX2.configPeakOutputVoltage(12.0f, -12.0f);
		rightSRX3.configPeakOutputVoltage(12.0f, -12.0f);
		leftSRX1.configPeakOutputVoltage(12.0f, -12.0f);
		leftSRX2.configPeakOutputVoltage(12.0f, -12.0f);
		leftSRX3.configPeakOutputVoltage(12.0f, -12.0f);
		rightSRX1.setVoltageRampRate(100);
		rightSRX2.setVoltageRampRate(100);
		rightSRX3.setVoltageRampRate(100);
		leftSRX1.setVoltageRampRate(100);
		leftSRX2.setVoltageRampRate(100);
		rightSRX3.setVoltageRampRate(100);
//		// shooter1.configPeakOutputVoltage(0, -8.0);
//		shooter2.configPeakOutputVoltage(0, -8.0);
		numberOfPWMisZeroinARow = 0;
		leftSRX2.changeControlMode(TalonControlMode.Follower);
		leftSRX2.set(leftSRX1.getDeviceID());
		rightSRX2.changeControlMode(TalonControlMode.Follower);
		rightSRX2.set(rightSRX1.getDeviceID());
		leftSRX3.changeControlMode(TalonControlMode.Follower);
		leftSRX3.set(leftSRX1.getDeviceID());
		rightSRX3.changeControlMode(TalonControlMode.Follower);
		rightSRX3.set(rightSRX1.getDeviceID());
//		// shooter1.changeControlMode(TalonControlMode.Speed);
//
//		shooter2.changeControlMode(TalonControlMode.Follower);
//		shooter2.set(// shooter1.getDeviceID());
		arm.setPosition(0);
		arm.setForwardSoftLimit(0.125);
		arm.enableForwardSoftLimit(false);
		arm.setReverseSoftLimit(0);
		arm.enableReverseSoftLimit(false);
		
		headingController.setInputRange(-180, 180);
		headingController.setContinuous(true);
		headingController.setOutputRange(-200, 200.0);
		headingController.setAbsoluteTolerance(2.0);
	}
	
	@Override
	public void disabledInit() {
		headingController.disable();
		leftSRX1.set(0);
		rightSRX1.set(0);
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString line to get the auto name from the text box below the Gyro
	 *
	 * You can add additional auto modes by adding additional comparisons to the
	 * switch structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	@Override
	public void autonomousInit()
	{
		autoCounter = 0;
		autoSelected = chooser.getSelected();
		System.out.println("Auto selected: " + autoSelected);
		navx.reset();
//		rightSRX1.changeControlMode(TalonControlMode.MotionProfile);
//		leftSRX1.changeControlMode(TalonControlMode.MotionProfile);
		arm.changeControlMode(TalonControlMode.Position);
		arm.setPosition(0);
		arm.set(0);
		timer.start();
		// shooter1.set(0);
	}

	/**
	 * This function is called periodically during autonomous
	 */

	int autoCounter = 0;
	
	@Override
	public void robotPeriodic() {
		SmartDashboard.putNumber("Pos Left", leftSRX1.getPosition());
		SmartDashboard.putNumber("Speed Left", leftSRX1.getSpeed());
		SmartDashboard.putNumber("Speed Right", rightSRX1.getSpeed());
		SmartDashboard.putNumber("Pos Right", rightSRX1.getPosition());
		SmartDashboard.putNumber("Heading", navx.getAngle());
		SmartDashboard.putNumber("ArmPosition", arm.getPosition());
		
	};

	@Override
	public void autonomousPeriodic()
	{
//		autoCounter++;
		
		headingController.enable();
		
//		rightSRX1.set(1);
//		leftSRX1.set(1);
//		rightSRX1.setF(4 / 1.5);
//		leftSRX1.setF(4 / 1.5);
//		leftSRX1.processMotionProfileBuffer();
//		rightSRX1.processMotionProfileBuffer();

		switch (autoSelected)
		{
		case rightGear:
			rightGearAuto();
			break;
		case middleGear:
			middleGearAuto();
			break;
		case sideShoot:
			sideShootAuto();
			break;
		case leftGear:
			leftGearAuto();
			break;
		}
	}

	public void rightGearAuto()
	{
		if (autoCounter == 1) // Drives forwards
		{
			addMotionProfileMotor(RightGearProfile.Points);
		}
		if (autoCounter == RightGearProfile.kNumPoints) // Turns around 30
														// degrees to the right
		{
			addMotionProfileMotor(RightGearProfile2.Points, rightSRX1);
		}
		if (autoCounter == RightGearProfile.kNumPoints + RightGearProfile2.kNumPoints + 25)// Goes
																							// forwards
																							// to
																							// peg
		{
			addMotionProfileMotor(RightGearProfile3.Points);
		}
		if (autoCounter == RightGearProfile.kNumPoints + RightGearProfile2.kNumPoints + RightGearProfile3.kNumPoints
				+ 50) // Goes backwards while outtaking and lowering arm
		{
			addMotionProfileMotor(RightGearProfile4.Points);
		}
		if (autoCounter > RightGearProfile.kNumPoints + RightGearProfile2.kNumPoints + RightGearProfile3.kNumPoints
				+ 75)
		{
			intake.set(-0.3);
		} else
		{
			intake.set(0);
		}
		if (autoCounter > RightGearProfile.kNumPoints + RightGearProfile2.kNumPoints + RightGearProfile3.kNumPoints + 75
				&& arm.getPosition() < 0.1)
		{
			arm.changeControlMode(TalonControlMode.PercentVbus);
			arm.set(-0.1);
		} else
		{
			arm.set(0);
		}
	}

	public void leftGearAuto()
	{
		if (autoCounter == 0)
		{
			resetDistances();
			drive(100.0, 0.0);
			autoCounter++;
			System.out.println("Starting middle gear auto");
		}
		if (autoCounter == 1 && getDistance() >= 2.0) {
			resetDistances();
			autoCounter++;
			drive(0.0, 60.0);
			System.out.println("Finished 1st drive");
		}
		if (autoCounter == 2 && headingController.onTarget()) {
			autoCounter++;
			drive(0.0, 60.0);
			System.out.println("Finished 2nd drive");
		}
		/*
		if (autoCounter == 1)
		{
			addMotionProfileMotor(RightGearProfile.Points);
		}
		if (autoCounter == RightGearProfile.kNumPoints) // Turns 30 degrees to
														// the left
		{
			addMotionProfileMotor(RightGearProfile2.Points, leftSRX1);
		}
		if (autoCounter == RightGearProfile.kNumPoints + RightGearProfile2.kNumPoints + 25)// Goes
																							// forwards
																							// to
																							// peg
		{
			addMotionProfileMotor(RightGearProfile3.Points);
		}
		if (autoCounter == RightGearProfile.kNumPoints + RightGearProfile2.kNumPoints + RightGearProfile3.kNumPoints
				+ 50) // Goes backwards while out-taking and lowering arm
		{
			addMotionProfileMotor(RightGearProfile4.Points);
		}
		if (autoCounter > RightGearProfile.kNumPoints + RightGearProfile2.kNumPoints + RightGearProfile3.kNumPoints
				+ 75)
		{
			intake.set(-0.3);
		} else
		{
			intake.set(0);
		}
		if (autoCounter > RightGearProfile.kNumPoints + RightGearProfile2.kNumPoints + RightGearProfile3.kNumPoints + 75
				&& arm.getPosition() < 0.1)
		{
			arm.changeControlMode(TalonControlMode.PercentVbus);
			arm.set(-0.1);
		} else
		{
			arm.set(0);
		}
		*/
	}
	
	public void resetDistances() {
		leftSRX1.setPosition(0.0);;
		rightSRX1.setPosition(0.0);;
	}
	
	public double getDistance() {
		return (leftSRX1.getPosition() + rightSRX1.getPosition()) / 2;
	}
	
	public void drive(double speed, double heading) {
		forwardSpeed = speed;
		headingController.setSetpoint(heading);
		headingController.enable();
		leftSRX1.enable();
		rightSRX1.enable();
		if (Math.abs(speed) < 10.0) {
			leftSRX1.clearIAccum();
			rightSRX1.clearIAccum();
		}
	}

	public void middleGearAuto()
	{
		if (autoCounter == 0)
		{
			resetDistances();
			drive(100.0, 0.0);
			autoCounter++;
			System.out.println("Starting middle gear auto");
		}
		if (autoCounter == 1 && getDistance() >= 2.0) {
			resetDistances();
			autoCounter++;
			drive(-100.0, 0.0);
			System.out.println("Finished 1st drive");
		}
		if (autoCounter == 2 && getDistance() <= -2.0) {
			autoCounter++;
			drive(0.0, 0.0);
			System.out.println("Finished 2nd drive");
		}
//		if (autoCounter > MiddleGearProfile.kNumPoints + 50 && arm.getPosition() < 0.08)
//		{
//			arm.changeControlMode(TalonControlMode.PercentVbus);
//			arm.set(-0.1);
//		} else
//		{
//			arm.set(0);
//		}
//		if (autoCounter == MiddleGearProfile.kNumPoints + 15)
//		{
//			addMotionProfileMotor(RightGearProfile4.Points);
//		}
//		if (autoCounter > MiddleGearProfile.kNumPoints + 50)
//		{
//			intake.set(-0.3);
//		} else
//		{
//			intake.set(0);
//		}
//		if (autoCounter > MiddleGearProfile.kNumPoints + 50 && arm.getPosition() < 0.1)
//		{
//			arm.changeControlMode(TalonControlMode.PercentVbus);
//			arm.set(-0.1);
//		} else
//		{
//			arm.set(0);
//		}

	}

	public void sideShootAuto()
	{
		if (autoCounter == 1)
		{
			leftSRX1.clearMotionProfileTrajectories();
			rightSRX1.clearMotionProfileTrajectories();
			rightSRX1.setPosition(0);
			leftSRX1.setPosition(0);
			leftSRX1.clearIAccum();
			rightSRX1.clearIAccum();
			CANTalon.TrajectoryPoint point = new CANTalon.TrajectoryPoint();
			for (int x = 0; x < MotionProfile.kNumPoints; x++)
			{
				point.position = MotionProfile.Points[x][0];
				point.velocity = MotionProfile.Points[x][1];
				point.timeDurMs = (int) MotionProfile.Points[x][2];
				point.velocityOnly = false;
				rightSRX1.pushMotionProfileTrajectory(point);
				point.position = MotionProfile.Points[x][0];
				point.velocity = MotionProfile.Points[x][1];
				leftSRX1.pushMotionProfileTrajectory(point);
			}
		}
		if (autoCounter == MotionProfile.kNumPoints + 25)
		{
			leftSRX1.clearMotionProfileTrajectories();
			rightSRX1.clearMotionProfileTrajectories();
			rightSRX1.setPosition(0);
			leftSRX1.setPosition(0);
			leftSRX1.clearIAccum();
			rightSRX1.clearIAccum();
			CANTalon.TrajectoryPoint point = new CANTalon.TrajectoryPoint();
			for (int x = 0; x < MotionProfile2.kNumPoints; x++)
			{
				point.position = MotionProfile2.Points[x][0];
				point.velocity = MotionProfile2.Points[x][1];
				point.timeDurMs = (int) MotionProfile2.Points[x][2];
				point.velocityOnly = false;
				rightSRX1.pushMotionProfileTrajectory(point);
				point.position = -MotionProfile2.Points[x][0];
				point.velocity = -MotionProfile2.Points[x][1];
				leftSRX1.pushMotionProfileTrajectory(point);
			}
		}
		if (autoCounter == MotionProfile.kNumPoints + MotionProfile2.kNumPoints + 50)
		{
			leftSRX1.clearMotionProfileTrajectories();
			rightSRX1.clearMotionProfileTrajectories();
			rightSRX1.setPosition(0);
			leftSRX1.setPosition(0);
			leftSRX1.clearIAccum();
			rightSRX1.clearIAccum();
			CANTalon.TrajectoryPoint point = new CANTalon.TrajectoryPoint();
			for (int x = 0; x < MotionProfile3.kNumPoints; x++)
			{
				point.position = MotionProfile3.Points[x][0];
				point.velocity = MotionProfile3.Points[x][1];
				point.timeDurMs = (int) MotionProfile3.Points[x][2];
				point.velocityOnly = false;
				rightSRX1.pushMotionProfileTrajectory(point);
				point.position = MotionProfile3.Points[x][0];
				point.velocity = MotionProfile3.Points[x][1];
				leftSRX1.pushMotionProfileTrajectory(point);
			}
		}
		if (autoCounter == MotionProfile.kNumPoints + MotionProfile2.kNumPoints + MotionProfile3.kNumPoints + 40)
		{
			leftSRX1.clearMotionProfileTrajectories();
			rightSRX1.clearMotionProfileTrajectories();
			rightSRX1.setPosition(0);
			leftSRX1.setPosition(0);
			leftSRX1.clearIAccum();
			rightSRX1.clearIAccum();
			CANTalon.TrajectoryPoint point = new CANTalon.TrajectoryPoint();
			for (int x = 0; x < MotionProfile4.kNumPoints; x++)
			{
				point.position = -1.9 * MotionProfile4.Points[x][0];
				point.velocity = -1.9 * MotionProfile4.Points[x][1];
				point.timeDurMs = (int) MotionProfile4.Points[x][2];
				point.velocityOnly = false;
				rightSRX1.pushMotionProfileTrajectory(point);
				point.position = 0.2 * MotionProfile4.Points[x][0];
				point.velocity = 0.2 * MotionProfile4.Points[x][1];
				leftSRX1.pushMotionProfileTrajectory(point);
			}
		}
		if (autoCounter >= MotionProfile.kNumPoints + MotionProfile2.kNumPoints + MotionProfile3.kNumPoints
				+ MotionProfile4.kNumPoints + 40)
		{
			// shooter1.changeControlMode(TalonControlMode.Speed);
			// shooter1.set(1200);
		}
//		if (shooter1.getSpeed() >= 1200)
//		{
//			index.set(1);
//			arm.set(0);
//		}
		// rightSRX1.set(1);
		// rightSRX1.processMotionProfileBuffer();
		SmartDashboard.putNumber("Position", rightSRX1.getPosition());
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopInit()
	{
//		// shooter1.changeControlMode(TalonControlMode.PercentVbus);
		arm.changeControlMode(TalonControlMode.PercentVbus);
		shooterToggle = false;
		intakeToggle = false;
		shooterSpeed = 600;
		rightSRX1.changeControlMode(TalonControlMode.Speed);
		leftSRX1.changeControlMode(TalonControlMode.Speed);
		arm.enableForwardSoftLimit(false);
		arm.enableReverseSoftLimit(false);
		CameraServer.getInstance().startAutomaticCapture();
	}

	public void teleopPeriodic()
	{
		getInputs(); // Gets joystick inputs
//		 // shooter1.setF(0.01);
//		 // shooter1.setPID(0.1, 0.003, 0.00000);
		if (buttonX2 != buttonXLast2 && buttonX2 && !shooterToggle) // Checks if
																	// button A
																	// was
																	// clicked
		{
			shooterToggle = !shooterToggle;
		} else if (buttonB2 != buttonBLast2 && buttonB2 && shooterToggle)
		{
			shooterToggle = !shooterToggle;
		}

		if (Math.abs(rightY2) > 0.2)
		{
			intake.set(rightY2);
		} else
		{
			intake.set(0);
		}
		arm.changeControlMode(TalonControlMode.PercentVbus);
		if (buttonY2)
		{
			placeGear();
		} else if (Math.abs(rightTrigger2 - leftTrigger2) > 0.2)
		{
			arm.set((rightTrigger2 - leftTrigger2) / 2.3);
		} else
		{
			arm.set(0.045);
		}
		if (buttonDPad1 != buttonDPadLast1 && buttonDPad1 != -1)
		{
			if (buttonDPad1 == 0)
			{
				shooterSpeed += 300;
			} else if (buttonDPad1 == 180 && shooterSpeed >= 300)
			{
				shooterSpeed -= 300;
			}
		}
		if (Math.abs(leftY1) < DeadZone)
			leftY1 = 0;
		if (Math.abs(rightX1) < DeadZone)
			rightX1 = 0;
		if (Math.abs(-leftY1 + rightX1) < DeadZone)
		{
			leftSRX1.ClearIaccum();
		}
		if (Math.abs(-leftY1 - rightX1) < DeadZone)
		{
			rightSRX1.ClearIaccum();
		}
		double leftSpeed;
		double rightSpeed;
		if (-leftY1 + rightX1 < 0)
		{
			leftSpeed = Math.pow(Math.abs(-leftY1 + rightX1), 2);
		} else
		{
			leftSpeed = -Math.pow(Math.abs(-leftY1 + rightX1), 2);
		}
		if (-leftY1 - rightX1 < 0)
		{
			rightSpeed = -Math.pow(Math.abs(-leftY1 - rightX1), 2);
		} else
		{
			rightSpeed = Math.pow(Math.abs(-leftY1 - rightX1), 2);
		}
		if (buttonDPad2 == 0)
		{
			leftSRX1.set(0);
			rightSRX1.changeControlMode(TalonControlMode.PercentVbus);
			rightSRX1.set(rightSpeed);
		} else
		{
			leftSRX1.changeControlMode(TalonControlMode.Speed);
			rightSRX1.changeControlMode(TalonControlMode.Speed);
			leftSRX1.set(leftSpeed * MaxRPM);
			rightSRX1.set(rightSpeed * MaxRPM);
		}
		if (buttonA1)
		{
			index.set(1);
		} else if (buttonX1)
		{
			index.set(-1);
		} else
		{
			index.set(0);
		}
		if (Math.abs(rightTrigger1 - leftTrigger1) > 0.2)
		{
			index.set(rightTrigger1 - leftTrigger1);
		} else
		{
			index.set(0);
		}
	//	SmartDashboard.putNumber("Shooter RPM", shooter1.getSpeed());
		if (shooterToggle)
		{
			// shooter1.changeControlMode(TalonControlMode.Speed);
			System.out.println("Shooting");
			// shooter1.set(shooterSpeed);
//			SmartDashboard.putNumber("Shooter Voltage",shooter1.getOutputVoltage());
		} else
		{
			// shooter1.set(0);
			// shooter1.clearIAccum();
		}
		// System.out.println("Left Encoder: " + leftSRX1.getEncPosition());
		// System.out.println("Right Encoder:" + rightSRX1.getEncPosition());
		getLastInputs();
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testInit()
	{
	}

	@Override

	public void testPeriodic()
	{
	}

	private void getLastInputs()
	{
		buttonALast1 = buttonA1;
		buttonBLast1 = buttonB1;
		buttonXLast1 = buttonX1;
		buttonYLast1 = buttonY1;
		buttonLBLast1 = buttonLB1;
		buttonRBLast1 = buttonRB1;
		buttonBackLast1 = buttonBack1;
		buttonStartLast1 = buttonStart1;
		buttonLeftStickClickLast1 = buttonLeftStickClick1;
		buttonRightStickClickLast1 = buttonRightStickClick1;
		buttonDPadLast1 = buttonDPad1;
		buttonALast2 = buttonA2;
		buttonBLast2 = buttonB2;
		buttonXLast2 = buttonX2;
		buttonYLast2 = buttonY2;
		buttonLBLast2 = buttonLB2;
		buttonRBLast2 = buttonRB2;
		buttonBackLast2 = buttonBack2;
		buttonStartLast2 = buttonStart2;
		buttonLeftStickClickLast2 = buttonLeftStickClick2;
		buttonRightStickClickLast2 = buttonRightStickClick2;
		buttonDPadLast2 = buttonDPad2;
	}

	private void getInputs()
	{
		leftX1 = joystick1.getRawAxis(0);
		leftY1 = joystick1.getRawAxis(1);
		rightX1 = joystick1.getRawAxis(4);
		rightY1 = joystick1.getRawAxis(5);
		leftTrigger1 = joystick1.getRawAxis(2);
		rightTrigger1 = joystick1.getRawAxis(3);
		buttonA1 = joystick1.getRawButton(1);
		buttonB1 = joystick1.getRawButton(2);
		buttonX1 = joystick1.getRawButton(3);
		buttonY1 = joystick1.getRawButton(4);
		buttonLB1 = joystick1.getRawButton(5);
		buttonRB1 = joystick1.getRawButton(6);
		buttonBack1 = joystick1.getRawButton(7);
		buttonStart1 = joystick1.getRawButton(8);
		buttonLeftStickClick1 = joystick1.getRawButton(9);
		buttonRightStickClick1 = joystick1.getRawButton(10);
		buttonDPad1 = joystick1.getPOV();
		leftX2 = joystick2.getRawAxis(0);
		leftY2 = joystick2.getRawAxis(2);
		rightX2 = joystick2.getRawAxis(4);
		rightY2 = joystick2.getRawAxis(5);
		leftTrigger2 = joystick2.getRawAxis(2);
		rightTrigger2 = joystick2.getRawAxis(3);
		buttonA2 = joystick2.getRawButton(1);
		buttonB2 = joystick2.getRawButton(2);
		buttonX2 = joystick2.getRawButton(3);
		buttonY2 = joystick2.getRawButton(4);
		buttonLB2 = joystick2.getRawButton(5);
		buttonRB2 = joystick2.getRawButton(6);
		buttonBack2 = joystick2.getRawButton(7);
		buttonStart2 = joystick2.getRawButton(8);
		buttonLeftStickClick2 = joystick2.getRawButton(9);
		buttonRightStickClick2 = joystick2.getRawButton(10);
		buttonDPad2 = joystick2.getPOV();
	}

	private void addMotionProfileMotor(double[][] points, CANTalon talon, boolean reverse)
	{
		talon.clearMotionProfileTrajectories();
		talon.setPosition(0);
		talon.clearIAccum();

		CANTalon.TrajectoryPoint point = new CANTalon.TrajectoryPoint();
		for (int x = 0; x < points.length; x++)
		{
			point.position = points[x][0];
			point.timeDurMs = (int) points[x][2];
			if (x != 0)
			{
				point.velocity = points[x][1]
						+ (points[x][1] - points[x - 1][1] * duration / 1000 * AccelerationCoefficient);
			} else
			{
				point.velocity = points[x][1];
			}
			point.velocityOnly = false;
			if (reverse)
			{
				point.velocity = -point.velocity;
				point.position = -point.position;
			}
			talon.pushMotionProfileTrajectory(point);
		}
	}

	private void addMotionProfileMotor(double[][] points, CANTalon talon) // defaults
																			// to
																			// not
																			// reversed
	{
		addMotionProfileMotor(points, talon, false);
	}

	private void addMotionProfileMotor(double[][] points) // Adds points to both
															// dirvetrain motors
	{
		leftSRX1.clearMotionProfileTrajectories();
		rightSRX1.clearMotionProfileTrajectories();
		rightSRX1.setPosition(0);
		leftSRX1.setPosition(0);
		leftSRX1.clearIAccum();
		rightSRX1.clearIAccum();
		CANTalon.TrajectoryPoint point = new CANTalon.TrajectoryPoint();
		for (int x = 0; x < points.length; x++)
		{
			point.position = points[x][0];
			point.timeDurMs = (int) points[x][2];
			if (x != 0)
			{
				point.velocity = points[x][1]
						+ (points[x][1] - points[x - 1][1] * duration / 1000 * AccelerationCoefficient);
			} else
			{
				point.velocity = points[x][1];
			}
			point.velocityOnly = false;
			leftSRX1.pushMotionProfileTrajectory(point);
			rightSRX1.pushMotionProfileTrajectory(point);
		}
	}

	private void placeGear()
	{
		arm.changeControlMode(TalonControlMode.PercentVbus);
		arm.set(-0.1);
		intake.set(-0.3);
	}
}
