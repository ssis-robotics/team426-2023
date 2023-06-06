// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.PowerDistribution;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.sensors.PigeonIMU;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private DifferentialDrive m_myRobot;
  private final PowerDistribution m_pdp = new PowerDistribution();
  private XboxController gamepadDrive;
  private double robotHeading;
  private DutyCycle aileronPWM;
  private DutyCycle elevationPWM;
  private double drivePWM;
  private double directionPWM;
  private double drive;
  private double direction;

  private WPI_TalonSRX leftMotorControllerCIM1;
  private WPI_TalonSRX leftMotorControllerCIM2;
  private WPI_TalonSRX rightMotorControllerCIM1;
  private WPI_TalonSRX rightMotorControllerCIM2;
  private MotorControllerGroup leftMotorGroup;
  private MotorControllerGroup rightMotorGroup;
  
  private WPI_TalonSRX climbMotorCIM1;
  private WPI_TalonSRX climbMotorCIM2;
  private WPI_VictorSPX conveyorMotorCIM1;
  private WPI_VictorSPX conveyorMotorCIM2;
  private WPI_VictorSPX colorWheelDrive;
  private WPI_VictorSPX colorWheelArm;

  private PigeonIMU pigeonIMU;
  private double [] pigeonIMUData;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    SmartDashboard.putData("PDP", m_pdp);
    leftMotorControllerCIM1 = new WPI_TalonSRX(10);
    leftMotorControllerCIM2 = new WPI_TalonSRX(1);
    leftMotorGroup = new MotorControllerGroup(leftMotorControllerCIM1,leftMotorControllerCIM2);
    rightMotorControllerCIM1 = new WPI_TalonSRX(2);
    rightMotorControllerCIM2 = new WPI_TalonSRX(3);
    rightMotorGroup = new MotorControllerGroup(rightMotorControllerCIM1,rightMotorControllerCIM2);
    // Create a differential drive system using the left and right motor groups
    m_myRobot = new DifferentialDrive(leftMotorGroup, rightMotorGroup);
    // Constructors for the other 6 motors
    climbMotorCIM1 = new WPI_TalonSRX(4);
    climbMotorCIM2 = new WPI_TalonSRX(5);
    conveyorMotorCIM1 = new WPI_VictorSPX(6);
    conveyorMotorCIM2 = new WPI_VictorSPX(7);
    colorWheelDrive = new WPI_VictorSPX(8);
    colorWheelArm = new WPI_VictorSPX(9);
    // Set up the two Xbox controllers. The drive is for driving, the operator is for all conveyor and color wheel controls
    gamepadDrive = new XboxController(0);

    aileronPWM = new DutyCycle(new DigitalInput(0));
    elevationPWM = new DutyCycle(new DigitalInput(1));
    
    pigeonIMU = new PigeonIMU(rightMotorControllerCIM2);
    pigeonIMUData = new double[3];
    pigeonIMU.setFusedHeading(70);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    double current0 = m_pdp.getCurrent(0);
    SmartDashboard.putNumber("Current Channel 0", current0);
    double voltage = m_pdp.getVoltage();
    SmartDashboard.putNumber("Voltage", voltage);
    SmartDashboard.putNumber("leftMotor", leftMotorControllerCIM1.get());
    SmartDashboard.putNumber("rightMotor", rightMotorControllerCIM1.get());
    pigeonIMU.getYawPitchRoll(pigeonIMUData);
    robotHeading = pigeonIMU.getFusedHeading();  
    SmartDashboard.putNumber("Robot Heading",robotHeading);
    SmartDashboard.putNumber("conveyorMotor1", conveyorMotorCIM1.get());
    SmartDashboard.putNumber("conveyorMotor2", conveyorMotorCIM2.get());
    SmartDashboard.putNumber("climbMotor1", climbMotorCIM1.get());
    SmartDashboard.putNumber("climbMotor2", climbMotorCIM2.get());
    drivePWM = (int) elevationPWM.getHighTimeNanoseconds()/1000;
    directionPWM = (int) aileronPWM.getHighTimeNanoseconds()/1000;
    SmartDashboard.putNumber("aileron time high", drivePWM);
    SmartDashboard.putNumber("elevation time high", directionPWM);
    drive = (drivePWM - 1500) / 500;
    direction = (directionPWM - 1500) / 500;
    SmartDashboard.putNumber("Drive", drive);
    SmartDashboard.putNumber("Direction", direction);
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    double leftX = gamepadDrive.getLeftX()*0.5; // from left controller
    double leftY = gamepadDrive.getLeftY()*1.0;
    drivePWM = (int) elevationPWM.getHighTimeNanoseconds()/1000;
    drive = (drivePWM - 1500) / 500;
    directionPWM = (int) aileronPWM.getHighTimeNanoseconds()/1000;
    direction = (directionPWM - 1500) / 500;    
    // combine left controller on Windows PC and PWM input
    m_myRobot.arcadeDrive(leftX + direction, leftY + drive);
    conveyorMotorCIM1.set(0.0);
    conveyorMotorCIM2.set(0.0);
    climbMotorCIM1.set(0.0);
    climbMotorCIM2.set(0);
    colorWheelArm.set(0);
    colorWheelDrive.set(0);
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
