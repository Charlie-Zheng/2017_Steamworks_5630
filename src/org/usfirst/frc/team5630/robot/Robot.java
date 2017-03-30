package org.usfirst.frc.team5630.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.buttons.Trigger;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Talon;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;
import com.ctre.CanTalonJNI.param_t;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	// Variable declarations
	final String defaultAuto = "Default";
	final String sideShootAuto = "Blue Shoot";
	final String middleGear = "Middle Gear";
	final String rightGear = "Right Gear";
	final double DeadZone = 0.215;
	int startEncoderTicks;
	double P = 3;
	double I = 0.02;
	double D = 0;
	double F = 0;
	String autoSelected;
	final int MaxRPM = 450;
	SendableChooser<String> chooser = new SendableChooser<>();
	CANTalon rightSRX3, shooter1, shooter2;
	CANTalon leftSRX3;
	CANTalon leftSRX1, rightSRX1, leftSRX2, rightSRX2;
	CANTalon intake, indexMotor, arm;
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

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		chooser.addDefault("Default Auto", defaultAuto);
		chooser.addObject(rightGear, rightGear);
		chooser.addObject(sideShootAuto, sideShootAuto);
		chooser.addObject(middleGear, middleGear);
		SmartDashboard.putData("Auto chooser", chooser);
		leftSRX1 = new CANTalon(1);
		leftSRX2 = new CANTalon(2);
		leftSRX3 = new CANTalon(3);
		shooter1 = new CANTalon(4);
		arm = new CANTalon(5);
		shooter2 = new CANTalon(6);
		rightSRX1 = new CANTalon(8);
		rightSRX2 = new CANTalon(7);
		rightSRX3 = new CANTalon(9);
		intake = new CANTalon(10);
		indexMotor = new CANTalon(11);
		joystick1 = new Joystick(0);
		joystick2 = new Joystick(1);
		// intakeMotor.setInverted(true);
		arm.setFeedbackDevice(FeedbackDevice.QuadEncoder);
		arm.configEncoderCodesPerRev(4096);
		shooter1.setFeedbackDevice(FeedbackDevice.QuadEncoder);
		shooter1.configEncoderCodesPerRev(4096);
		leftSRX1.setFeedbackDevice(FeedbackDevice.QuadEncoder);
		leftSRX1.configEncoderCodesPerRev(180);
		leftSRX1.enableBrakeMode(true);
		leftSRX2.enableBrakeMode(true);
		leftSRX3.enableBrakeMode(true);
		// System.out.println("F is: " + leftSRX1.getF());
		leftSRX1.setPID(1, 0.0023, 0);
		leftSRX1.reverseSensor(true);
		leftSRX1.setInverted(true);
		leftSRX1.reverseOutput(false);
		leftSRX2.reverseOutput(false);
		leftSRX3.reverseOutput(false);
		rightSRX1.setFeedbackDevice(FeedbackDevice.QuadEncoder);
		rightSRX1.configEncoderCodesPerRev(180);
		rightSRX1.setPID(1.2, 0.003, 0);
		rightSRX1.reverseSensor(false);
		rightSRX1.setInverted(false);
		rightSRX1.reverseOutput(true);
		rightSRX2.reverseOutput(false);
		rightSRX3.reverseOutput(false);
		arm.setInverted(true);
		arm.reverseSensor(false);
		arm.configEncoderCodesPerRev(4096);
		arm.setForwardSoftLimit(0);
		arm.setReverseSoftLimit(-0.127);
		arm.setPID(0.1, 0.0001, 0);
		arm.setF(100);
		intake.setInverted(false);
		indexMotor.setInverted(false);

		shooter1.setInverted(false);
		shooter2.setInverted(false);
		shooter1.reverseSensor(false);
		arm.configPeakOutputVoltage(4, -4);
		rightSRX3.configPeakOutputVoltage(12.0f, -12.0f);
		rightSRX1.configPeakOutputVoltage(12.0f, -12.0f);
		rightSRX2.configPeakOutputVoltage(12.0f, -12.0f);
		leftSRX3.configPeakOutputVoltage(12.0f, -12.0f);
		leftSRX1.configPeakOutputVoltage(12.0f, -12.0f);
		leftSRX2.configPeakOutputVoltage(12.0f, -12.0f);
		rightSRX1.setVoltageRampRate(36);
		rightSRX2.setVoltageRampRate(36);
		rightSRX3.setVoltageRampRate(36);
		leftSRX1.setVoltageRampRate(36);
		leftSRX2.setVoltageRampRate(36);
		leftSRX3.setVoltageRampRate(36);
		shooter1.configPeakOutputVoltage(8.0f, 0.0f);
		shooter2.configPeakOutputVoltage(8.0f, 0.0f);
		numberOfPWMisZeroinARow = 0;
		leftSRX3.changeControlMode(TalonControlMode.Follower);
		leftSRX2.changeControlMode(TalonControlMode.Follower);
		leftSRX3.set(leftSRX1.getDeviceID());
		leftSRX2.set(leftSRX1.getDeviceID());
		rightSRX3.changeControlMode(TalonControlMode.Follower);
		rightSRX2.changeControlMode(TalonControlMode.Follower);
		rightSRX3.set(rightSRX1.getDeviceID());
		rightSRX2.set(rightSRX1.getDeviceID());
		shooter1.changeControlMode(TalonControlMode.Speed);
		shooter2.changeControlMode(TalonControlMode.Follower);
		shooter2.set(shooter1.getDeviceID());
		arm.setPosition(0);
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
	public void autonomousInit() {
		autoCounter = 0;
		SmartDashboard.putData("Auto chooser", chooser);
		autoSelected = chooser.getSelected();
		// autoSelected = SmartDashboard.getString("Auto Selector",
		// defaultAuto);
		System.out.println("Auto selected: " + autoSelected);
		rightSRX1.changeControlMode(TalonControlMode.MotionProfile);
		leftSRX1.changeControlMode(TalonControlMode.MotionProfile);
	}

	/**
	 * This function is called periodically during autonomous
	 */

	int autoCounter = 0;

	@Override
	public void autonomousPeriodic() {
		autoCounter++;
		SmartDashboard.putNumber("Pos Left", leftSRX1.getPosition());
		SmartDashboard.putNumber("Speed Left", leftSRX1.getSpeed());
		SmartDashboard.putNumber("Speed Right", rightSRX1.getSpeed());
		SmartDashboard.putNumber("Pos Right", rightSRX1.getPosition());
		arm.setPosition(0);
		rightSRX1.set(1);
		leftSRX1.set(1);
		rightSRX1.setF(4.5/1.5);
		leftSRX1.setF(2.75/1.5);
		leftSRX1.processMotionProfileBuffer();
		rightSRX1.processMotionProfileBuffer();
		switch (autoSelected) {
		case rightGear:

			if (autoCounter == 1) {
				leftSRX1.clearMotionProfileTrajectories();
				rightSRX1.clearMotionProfileTrajectories();
				rightSRX1.setPosition(0);
				leftSRX1.setPosition(0);
				leftSRX1.clearIAccum();
				rightSRX1.clearIAccum();
				CANTalon.TrajectoryPoint point = new CANTalon.TrajectoryPoint();
				for (int x = 0; x < RightGearProfile.kNumPoints; x++) {
					point.position = RightGearProfile.Points[x][0];
					point.velocity = RightGearProfile.Points[x][1];
					point.timeDurMs = (int) RightGearProfile.Points[x][2];
					point.velocityOnly = false;
					rightSRX1.pushMotionProfileTrajectory(point);
					point.position = RightGearProfile.Points[x][0];
					point.velocity = RightGearProfile.Points[x][1];
					leftSRX1.pushMotionProfileTrajectory(point);
				}
			}
			if(autoCounter == RightGearProfile.kNumPoints){
				leftSRX1.clearMotionProfileTrajectories();
				rightSRX1.clearMotionProfileTrajectories();
				leftSRX1.clearIAccum();
				rightSRX1.clearIAccum();
				CANTalon.TrajectoryPoint point = new CANTalon.TrajectoryPoint();
				for (int x = 0; x < RightGearProfile2.kNumPoints; x++) {
					point.position = RightGearProfile.Points[RightGearProfile.kNumPoints-1][0]+RightGearProfile2.Points[x][0];
					point.velocity = RightGearProfile2.Points[x][1];
					point.timeDurMs = (int) RightGearProfile2.Points[x][2];
					point.velocityOnly = false;
					rightSRX1.pushMotionProfileTrajectory(point);
				}
			}
			if(autoCounter == RightGearProfile.kNumPoints + RightGearProfile2.kNumPoints + 75)	{
				leftSRX1.clearMotionProfileTrajectories();
				rightSRX1.clearMotionProfileTrajectories();
				leftSRX1.clearIAccum();
				rightSRX1.clearIAccum();
				CANTalon.TrajectoryPoint point = new CANTalon.TrajectoryPoint();
				for (int x = 0; x < RightGearProfile3.kNumPoints; x++) {
					point.position = RightGearProfile.Points[RightGearProfile.kNumPoints-1][0]+RightGearProfile2.Points[RightGearProfile2.kNumPoints-1][0]+RightGearProfile3.Points[x][0];
					point.velocity = RightGearProfile3.Points[x][1];
					point.timeDurMs = (int) RightGearProfile3.Points[x][2];
					point.velocityOnly = false;
					rightSRX1.pushMotionProfileTrajectory(point);
					point.position = RightGearProfile.Points[RightGearProfile.kNumPoints-1][0]+RightGearProfile3.Points[x][0];
					point.velocity = RightGearProfile3.Points[x][1];
					leftSRX1.pushMotionProfileTrajectory(point);
				}
			}
//			if(autoCounter == RightGearProfile.kNumPoints+RightGearProfile2.kNumPoints){
//				leftSRX1.clearMotionProfileTrajectories();
//				rightSRX1.clearMotionProfileTrajectories();
//				leftSRX1.clearIAccum();
//				rightSRX1.clearIAccum();
//				CANTalon.TrajectoryPoint point = new CANTalon.TrajectoryPoint();
//				for (int x = 0; x < RightGearProfile2.kNumPoints; x++) {
//					point.position = RightGearProfile.Points[RightGearProfile.kNumPoints-1][0]+RightGearProfile2.Points[x][0];
//					point.velocity = RightGearProfile2.Points[x][1];
//					point.timeDurMs = (int) RightGearProfile2.Points[x][2];
//					point.velocityOnly = false;
//					rightSRX1.pushMotionProfileTrajectory(point);
//				}
//			}
			break;
		case middleGear:
			// Put custom auto code here
			if (autoCounter == 1) {
				leftSRX1.clearMotionProfileTrajectories();
				rightSRX1.clearMotionProfileTrajectories();
				rightSRX1.setPosition(0);
				leftSRX1.setPosition(0);
				leftSRX1.clearIAccum();
				rightSRX1.clearIAccum();
				CANTalon.TrajectoryPoint point = new CANTalon.TrajectoryPoint();
				for (int x = 0; x < MiddleGearProfile.kNumPoints; x++) {
					point.position = MiddleGearProfile.Points[x][0];
					point.velocity = MiddleGearProfile.Points[x][1];
					point.timeDurMs = (int) MiddleGearProfile.Points[x][2];
					point.velocityOnly = true;
					rightSRX1.pushMotionProfileTrajectory(point);
					point.position = MiddleGearProfile.Points[x][0];
					point.velocity = MiddleGearProfile.Points[x][1];
					leftSRX1.pushMotionProfileTrajectory(point);
				}
			}
			if (autoCounter == MiddleGearProfile.kNumPoints + 15) {
				arm.changeControlMode(TalonControlMode.MotionProfile);
				arm.clearMotionProfileTrajectories();
				CANTalon.TrajectoryPoint point = new CANTalon.TrajectoryPoint();
				for (int x = 0; x < GearArmProfile.kNumPoints; x++) {
					point.position = GearArmProfile.Points[x][0];
					point.velocity = GearArmProfile.Points[x][1];
					point.timeDurMs = (int) GearArmProfile.Points[x][2];
					point.velocityOnly = false;
					arm.pushMotionProfileTrajectory(point);
				}
			}
			if (autoCounter > MiddleGearProfile.kNumPoints + 15) {
				intake.set(-0.5);
			}

			break;
		case sideShootAuto:
		default:
			// Put default auto code here
			if (autoCounter == 1) {
				leftSRX1.clearMotionProfileTrajectories();
				rightSRX1.clearMotionProfileTrajectories();
				rightSRX1.setPosition(0);
				leftSRX1.setPosition(0);
				leftSRX1.clearIAccum();
				rightSRX1.clearIAccum();
				CANTalon.TrajectoryPoint point = new CANTalon.TrajectoryPoint();
				for (int x = 0; x < MotionProfile.kNumPoints; x++) {
					point.position = MotionProfile.Points[x][0];
					point.velocity = MotionProfile.Points[x][1];
					point.timeDurMs = (int) MotionProfile.Points[x][2];
					point.velocityOnly = false;
					rightSRX1.pushMotionProfileTrajectory(point);
					point.position = MotionProfile.Points[x][0];
					point.velocity = MotionProfile.Points[x][1];
					leftSRX1.pushMotionProfileTrajectory(point);
				}
				// rightSRX1.setPID(1, 0.000, 0);
				// leftSRX1.setPID(1, 0.000, 0);
			}
			if (autoCounter == MotionProfile.kNumPoints + 75) {
				leftSRX1.clearMotionProfileTrajectories();
				rightSRX1.clearMotionProfileTrajectories();
				rightSRX1.setPosition(0);
				leftSRX1.setPosition(0);
				leftSRX1.clearIAccum();
				rightSRX1.clearIAccum();
				CANTalon.TrajectoryPoint point = new CANTalon.TrajectoryPoint();
				for (int x = 0; x < MotionProfile2.kNumPoints; x++) {
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
			if (autoCounter == MotionProfile.kNumPoints + MotionProfile2.kNumPoints + 150) {
				leftSRX1.clearMotionProfileTrajectories();
				rightSRX1.clearMotionProfileTrajectories();
				rightSRX1.setPosition(0);
				leftSRX1.setPosition(0);
				leftSRX1.clearIAccum();
				rightSRX1.clearIAccum();
				CANTalon.TrajectoryPoint point = new CANTalon.TrajectoryPoint();
				for (int x = 0; x < MotionProfile3.kNumPoints; x++) {
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
			if (autoCounter == MotionProfile.kNumPoints + MotionProfile2.kNumPoints + MotionProfile3.kNumPoints + 150) {
				leftSRX1.clearMotionProfileTrajectories();
				rightSRX1.clearMotionProfileTrajectories();
				rightSRX1.setPosition(0);
				leftSRX1.setPosition(0);
				leftSRX1.clearIAccum();
				rightSRX1.clearIAccum();
				CANTalon.TrajectoryPoint point = new CANTalon.TrajectoryPoint();
				for (int x = 0; x < MotionProfile4.kNumPoints; x++) {
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
			// rightSRX1.set(1);
			// rightSRX1.processMotionProfileBuffer();
			SmartDashboard.putNumber("Position", rightSRX1.getPosition());
			break;
		}
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopInit() {
		// rightSRX.changeControlMode(TalonControlMode.PercentVbus); // Changes
		// to
		// % Voltage
		shooter1.changeControlMode(TalonControlMode.PercentVbus);
		arm.changeControlMode(TalonControlMode.PercentVbus);
		shooterToggle = false;
		intakeToggle = false;
		shooterSpeed = 1200;
		leftSRX3.changeControlMode(TalonControlMode.Follower);
		leftSRX2.changeControlMode(TalonControlMode.Follower);
		leftSRX3.set(leftSRX1.getDeviceID());
		leftSRX2.set(leftSRX1.getDeviceID());
		rightSRX3.changeControlMode(TalonControlMode.Follower);
		rightSRX2.changeControlMode(TalonControlMode.Follower);
		rightSRX3.set(rightSRX1.getDeviceID());
		rightSRX2.set(rightSRX1.getDeviceID());
		rightSRX1.changeControlMode(TalonControlMode.Speed);
		leftSRX1.changeControlMode(TalonControlMode.Speed);
	}

	public void teleopPeriodic() {
		// double start = System.currentTimeMillis();
		getInputs(); // Gets joystick inputs
		shooter1.setF(0.06);
		shooter1.setPID(0.1, 0.00003, 0.00000);
		SmartDashboard.putNumber("ArmPosition", arm.getPosition());
		SmartDashboard.putNumber("Speed", rightSRX1.getSpeed());
		SmartDashboard.putNumber("EncSpeed", rightSRX1.getEncVelocity());
		SmartDashboard.putNumber("RightX1", rightX1);
		SmartDashboard.putNumber("right output", rightSRX1.getOutputVoltage());
		if (buttonB1 != buttonBLast1 && buttonB1) {// Checks if button A was
			// clicked shooterToggle =
			// !shooterToggle;
			// System.out.println("ShooterToggle: " + shooterToggle);
			shooterToggle = !shooterToggle;
		}
		// if (buttonRB2 != buttonRBLast2 && buttonRB2) {
		// intakeToggle = !intakeToggle;
		// }
		if (buttonLB1) {
			intake.set(1);
		} else if (buttonRB1) {
			intake.set(-1);
		} else {
			intake.set(0);
		}

		//
		if (Math.abs(rightTrigger1 - leftTrigger1) > 0.2) {
			arm.changeControlMode(TalonControlMode.PercentVbus);
			arm.set((rightTrigger1 - leftTrigger1) / 3);
		} else {
			arm.set(0.033);
		}

		/*
		 * if(buttonRB = true){//Checks if button A was clicked
		 * indexMotor.set(1); }
		 */

		// intakeMotor.set(rightTrigger);//Runs intake with right trigger speed

		/*
		 * if(shooterToggle){ shooter1.set(0.4); shooter2.set(0.4); }else{
		 * shooter1.set(0); shoot
		 */

		// climberMotor.set(leftTrigger);
		if (buttonDPad1 != buttonDPadLast1 && buttonDPad1 != -1) {
			if (buttonDPad1 == 0) {
				shooterSpeed += 300;
			} else if (buttonDPad1 == 180 && shooterSpeed >= 300) {
				shooterSpeed -= 300;
			}
		}
		if (Math.abs(leftY1) < DeadZone)
			leftY1 = 0;
		if (Math.abs(rightX1) < DeadZone)
			rightX1 = 0;
		if (Math.abs(-leftY1 + rightX1) < DeadZone) {
			leftSRX1.ClearIaccum();
		}
		if (Math.abs(-leftY1 - rightX1) < DeadZone) {
			rightSRX1.ClearIaccum();
		}
		double leftSpeed;
		double rightSpeed;
		if (-leftY1 + rightX1 < 0) {
			leftSpeed = Math.pow(Math.abs(-leftY1 + rightX1), 2);
		} else {
			leftSpeed = -Math.pow(Math.abs(-leftY1 + rightX1), 2);
		}
		if (-leftY1 - rightX1 < 0) {
			rightSpeed = -Math.pow(Math.abs(-leftY1 - rightX1), 2);
		} else {
			rightSpeed = Math.pow(Math.abs(-leftY1 - rightX1), 2);
		}
		leftSRX1.set(leftSpeed * MaxRPM);
		rightSRX1.set(rightSpeed * MaxRPM);
		// System.out.println();
		if (buttonA1) {
			indexMotor.set(1);
		} else {
			indexMotor.set(0);
		}
		if (shooterToggle) {
			shooter1.changeControlMode(TalonControlMode.Speed);
			shooter1.set(shooterSpeed);
			SmartDashboard.putNumber("Shooter RPM", shooter1.getSpeed());
			SmartDashboard.putNumber("Shooter VOltage", shooter1.getOutputVoltage());
		} else {
			shooter1.set(0);
			shooter1.clearIAccum();
		}
		// if(intakeToggle){
		// intakeMotor.set(1);
		// }else{
		// intakeMotor.set(0);
		// }
		// if (buttonRB1) {
		// indexMotor.set(0.5);
		// } else {
		// indexMotor.set(0);
		// }
		// if (buttonLB1) {
		// intakeMotor.set(1);
		// } else if (buttonRB1) {
		// intakeMotor.set(-1);
		// } else {
		// intakeMotor.set(0);
		// }

		// leftMotor.set(leftSRX.getOutputVoltage()/leftSRX.getBusVoltage());
		// rightMotor.set(rightSRX.getOutputVoltage()/rightSRX.getBusVoltage());
		// System.out.println("Output of left SRX: " +
		// leftSRX.getOutputVoltage() + "\t Output of right SRX: " +
		// rightSRX.getOutputVoltage());
		// System.out.println("PWM of left: " +
		// leftSRX.getOutputVoltage()/leftSRX.getBusVoltage() + "PWM of right: "
		// + rightSRX.getOutputVoltage()/rightSRX.getBusVoltage());

		// if (buttonDPad1 != buttonDPadLast1 && buttonDPad1 != -1) {
		// if (buttonDPad1 == 0) {
		// I += 0.001;
		// } else if (buttonDPad1 == 180 && I > 0) {
		// I -= 0.001;
		// if (I < 0) {
		// I = 0;
		// }
		//
		// }
		// leftSRX1.setPID(P, I, D);
		// rightSRX1.setPID(P, I, D);
		//
		// System.out.println("I is now: " + leftSRX1.getI());
		// }
		// if (buttonDPad1 != buttonDPadLast1 && buttonDPad1 != -1) {
		// if (buttonDPad1 == 90) {
		// P += 0.01;
		// } else if (buttonDPad1 == 270 && P > 0) {
		// P -= 0.01;
		// if (P < 0) {
		// P = 0;
		// }
		//
		// }
		// leftSRX1.setPID(P, I, D);
		// rightSRX1.setPID(P, I, D);
		//
		// System.out.println("P is now: " + leftSRX1.getP());
		// }
		getLastInputs();
		// System.out.println("Time for one cycle: " +
		// (System.currentTimeMillis()-start));

	}

	/**
	 * This function is called periodically during test mode
	 */

	public void testInit() {
		startEncoderTicks = leftSRX1.getEncPosition();
		leftSRX1.setPosition(0);
		// shooter1.changeControlMode(TalonControlMode.Speed);
		leftSRX3.changeControlMode(TalonControlMode.Follower);
		leftSRX2.changeControlMode(TalonControlMode.Follower);
		leftSRX3.set(leftSRX1.getDeviceID());
		leftSRX2.set(leftSRX1.getDeviceID());
		rightSRX3.changeControlMode(TalonControlMode.Follower);
		rightSRX2.changeControlMode(TalonControlMode.Follower);
		rightSRX3.set(rightSRX1.getDeviceID());
		rightSRX2.set(rightSRX1.getDeviceID());
		rightSRX1.changeControlMode(TalonControlMode.Speed);
		leftSRX1.changeControlMode(TalonControlMode.Speed);
		SmartDashboard.putNumber("Speed", leftSRX1.getSpeed());
	}

	@Override
	public void testPeriodic() {
		getInputs();
		SmartDashboard.putNumber("Speed", leftSRX1.getSpeed());
		// System.out.println("Speed: " + rightSRX1.getSpeed());
		leftSRX3.set(leftSRX1.getDeviceID());
		robotDrive.arcadeDrive(-leftY1, -rightX1);
		// System.out.println("SRX#3 Output Voltage: " +
		// leftSRX3.getOutputVoltage());
		// if (buttonA1) {
		// leftSRX1.changeControlMode(TalonControlMode.PercentVbus);
		// leftSRX1.set(0.3);
		// } else {
		// leftSRX1.set(0);
		// }
		// if (buttonB1) {
		// leftSRX2.changeControlMode(TalonControlMode.PercentVbus);
		// leftSRX2.set(0.3);
		// } else {
		// leftSRX2.set(0);
		// }
		// if (buttonY1) {
		// leftSRX3.changeControlMode(TalonControlMode.PercentVbus);
		// leftSRX3.set(0.3);
		// } else {
		// leftSRX3.set(0);
		// }

		// // System.out.println("Encoder ticks: " + (shooter1.getEncPosition()
		// -
		// // startEncoderTicks));
		//
		if (buttonDPad1 != buttonDPadLast1 && buttonDPad1 != -1) {
			if (buttonDPad1 == 0) {
				I += 0.01;
			} else if (buttonDPad1 == 180 && I > 0) {
				I -= 0.01;
				if (I < 0) {
					I = 0;
				}

			}
			leftSRX1.setPID(P, I, D);
			rightSRX1.setPID(P, I, D);

			System.out.println("P is now: " + leftSRX1.getP());
		}
		//
		// if (buttonA1) {
		// shooter1.changeControlMode(TalonControlMode.Speed);
		// shooter1.set(1500);
		// System.out.println("Shooter speed is: " + shooter1.getSpeed());
		// } else if (buttonY1) {
		// shooter1.changeControlMode(TalonControlMode.PercentVbus);
		// shooter1.set(0.35);
		// } else {
		// shooter1.set(0);
		//
		// }
		// if (buttonB1) {
		// System.out.println(shooter1.getSpeed());
		// }
		getLastInputs();
	}

	private void getLastInputs() {
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

	private void getInputs() {
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
}
