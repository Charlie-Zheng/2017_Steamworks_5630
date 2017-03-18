package org.usfirst.frc.team5630.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Talon;

import com.ctre.CANTalon;
import com.ctre.CANTalon.TalonControlMode;

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
	final String customAuto = "My Auto";
	String autoSelected;
	SendableChooser<String> chooser = new SendableChooser<>();
	CANTalon rightSRX0, shooter1, shooter2;
	CANTalon leftSRX0;
	CANTalon leftSRX1, rightSRX1, leftSRX2, rightSRX2;
	CANTalon intakeMotor, indexMotor, armMotor;
	Joystick joystick1, joystick2;
	double rightX1, rightY1, leftTrigger1, rightTrigger1, leftX1, leftY1;
	boolean buttonA1, buttonB1, buttonX1, buttonY1, buttonRB1, buttonLB1, buttonLeftStickClick1, buttonRightStickClick1,
			buttonBack1, buttonStart1;
	boolean buttonALast1, buttonBLast1, buttonXLast1, buttonYLast1, buttonRBLast1, buttonLBLast1,
			buttonLeftStickClickLast1, buttonRightStickClickLast1, buttonBackLast1, buttonStartLast1;
	double rightX2, rightY2, leftTrigger2, rightTrigger2, leftX2, leftY2;
	boolean buttonA2, buttonB2, buttonX2, buttonY2, buttonRB2, buttonLB2, buttonLeftStickClick2, buttonRightStickClick2,
			buttonBack2, buttonStart2;
	int buttonDPad2;
	boolean buttonALast2, buttonBLast2, buttonXLast2, buttonYLast2, buttonRBLast2, buttonLBLast2,
			buttonLeftStickClickLast2, buttonRightStickClickLast2, buttonBackLast2, buttonStartLast2;
	int buttonDPadLast2;
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
		chooser.addObject("My Auto", customAuto);

		SmartDashboard.putData("Auto choices", chooser);
		leftSRX1 = new CANTalon(1);
		leftSRX2 = new CANTalon(2);
		leftSRX0 = new CANTalon(3);
		shooter1 = new CANTalon(4);
		shooter2 = new CANTalon(5);
		rightSRX2 = new CANTalon(6);
		rightSRX1 = new CANTalon(7);
		rightSRX0 = new CANTalon(8);
		intakeMotor = new CANTalon(9);
		indexMotor = new CANTalon(10);
		armMotor = new CANTalon(11);
		joystick1 = new Joystick(0);
		joystick2 = new Joystick(1);
		// intakeMotor.setInverted(true);
		intakeMotor.setInverted(true);
		indexMotor.setInverted(true);
		shooter1.setInverted(false);
		shooter2.setInverted(false);
		leftSRX0.setInverted(false);
		leftSRX1.setInverted(false);
		leftSRX2.setInverted(false);
		rightSRX0.setInverted(false);
		rightSRX1.setInverted(false);
		rightSRX2.setInverted(false);
		rightSRX0.configPeakOutputVoltage(12.0f, -12.0f);
		rightSRX1.configPeakOutputVoltage(12.0f, -12.0f);
		rightSRX2.configPeakOutputVoltage(12.0f, -12.0f);
		leftSRX0.configPeakOutputVoltage(12.0f, -12.0f);
		leftSRX1.configPeakOutputVoltage(12.0f, -12.0f);
		leftSRX2.configPeakOutputVoltage(12.0f, -12.0f);
		numberOfPWMisZeroinARow = 0;
		robotDrive = new RobotDrive(leftSRX0, rightSRX0);
		leftSRX1.changeControlMode(TalonControlMode.Follower);
		leftSRX2.changeControlMode(TalonControlMode.Follower);
		leftSRX1.set(leftSRX0.getDeviceID());
		leftSRX2.set(leftSRX0.getDeviceID());
		rightSRX1.changeControlMode(TalonControlMode.Follower);
		rightSRX2.changeControlMode(TalonControlMode.Follower);
		rightSRX1.set(rightSRX0.getDeviceID());
		rightSRX2.set(rightSRX0.getDeviceID());
		shooter2.changeControlMode(TalonControlMode.Follower);
		shooter2.set(shooter1.getDeviceID());
		// shooter1.setInverted(false);
		// shooter2.setInverted(false);
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
		autoSelected = chooser.getSelected();
		// autoSelected = SmartDashboard.getString("Auto Selector",
		// defaultAuto);
		System.out.println("Auto selected: " + autoSelected);
		 
		
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		switch (autoSelected) {
		case customAuto:
			// Put custom auto code here
			break;
		case defaultAuto:
		default:
			// Put default auto code here
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
		leftSRX0.changeControlMode(TalonControlMode.PercentVbus);
		shooterToggle = false;
		intakeToggle = false;
		// robotDrive = new RobotDrive(leftSRX, leftMotor, rightSRX,
		// rightMotor);// front
		// left,
		// rear
		// left,
		// front
		// right,
		// rear
		// right

		shooterSpeed = 0.5;
	}

	@Override
	public void teleopPeriodic() {
		// double start = System.currentTimeMillis();
		getInputs(); // Gets joystick inputs

		if (buttonA2 != buttonALast2 && buttonA2) {// Checks if button A was
													// clicked shooterToggle =
													// !shooterToggle;
			// System.out.println("ShooterToggle: " + shooterToggle);
			shooterToggle = !shooterToggle;
		}
		if (buttonRB2 != buttonRBLast2 && buttonRB2) {
			intakeToggle = !intakeToggle;
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
		if (buttonDPad2 != buttonDPadLast2 && buttonDPad2 != -1) {
			if (buttonDPad2 == 0 && shooterSpeed != 0.6) {
				shooterSpeed += 0.05;
			} else if (buttonDPad2 == 180 && shooterSpeed != 0.1) {
				shooterSpeed -= 0.05;
			}
		}

		robotDrive.arcadeDrive(-leftY1, rightX1);
		// System.out.println();
		if (shooterToggle) {
			shooter1.set(shooterSpeed);
			System.out.println(shooterSpeed);
		} else {
			shooter1.set(0);
		}
		// if(intakeToggle){
		// intakeMotor.set(1);
		// }else{
		// intakeMotor.set(0);
		// }
		intakeMotor.set(rightTrigger2);
		indexMotor.set(leftTrigger2);
		// leftMotor.set(leftSRX.getOutputVoltage()/leftSRX.getBusVoltage());
		// rightMotor.set(rightSRX.getOutputVoltage()/rightSRX.getBusVoltage());
		// System.out.println("Output of left SRX: " +
		// leftSRX.getOutputVoltage() + "\t Output of right SRX: " +
		// rightSRX.getOutputVoltage());
		// System.out.println("PWM of left: " +
		// leftSRX.getOutputVoltage()/leftSRX.getBusVoltage() + "PWM of right: "
		// + rightSRX.getOutputVoltage()/rightSRX.getBusVoltage());

		getLastInputs();
		// System.out.println("Time for one cycle: " +
		// (System.currentTimeMillis()-start));
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {

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
