// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.filter.SlewRateLimiter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the manifest file in the resource
 * directory.
 */
/** RIGHT_DRIVE_1    01 Talon SRX rightDrive1
 *  RIGHT_DRIVE_2    02 Talon SRX rightDrive2Follower
 *  LEFT_DRIVE_3     03 Talon SRX leftDrive3
 *  LEFT_DRIVE_4     04 Talon SRX leftDrive4Follower
 *  RIGHT_CONVEYER_5 05 Spark Max
 *  LEFT_CONVEYER_6  06 Spark Max
 */
public class Robot extends TimedRobot {
  
  private final WPI_TalonSRX rightDrive1 = new WPI_TalonSRX(1);
  private final WPI_TalonSRX rightDrive2Follower = new WPI_TalonSRX(2);
  private final WPI_TalonSRX leftDrive3 = new WPI_TalonSRX(3);
  private final WPI_TalonSRX leftDrive4Follower = new WPI_TalonSRX(4);
  private final CANSparkMax rightIntake = new CANSparkMax(5, MotorType.kBrushless);
  private final CANSparkMax leftIntake = new CANSparkMax(6, MotorType.kBrushless);
  private final CANSparkMax climber = new CANSparkMax(7, MotorType.kBrushless);
  private final WPI_TalonSRX roller = new WPI_TalonSRX(8);
  private final DifferentialDrive robotDrive = new DifferentialDrive(leftDrive3, rightDrive1);
  private final XboxController driverController = new XboxController(0);
  private final XboxController operatorController = new XboxController(1);
  private int autonCounter = 0;  

  private final Timer autonTimer = new Timer();
  private double QUARTERSPEED = 0.25;
  private double HALFSPEED = 0.5;
  private double FULLSPEED = 1;
  private double intakeSpeed = FULLSPEED;
  private double climberSpeed = QUARTERSPEED;
  boolean leftToggle = true;
  boolean rightToggle = true;
  boolean rollerToggle = false; 
  boolean driveToggle = false;
  /** parameter is 5 units per second, accelerate 0 to 1 in 1/5 of a secocnd */
  SlewRateLimiter driveLimiter = new SlewRateLimiter(3);
  SlewRateLimiter turnLimter = new SlewRateLimiter(10);
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
   
  
    /* Check? Are Factory defaults needed? */
    rightDrive1.configFactoryDefault();
    rightDrive2Follower.configFactoryDefault();
    leftDrive3.configFactoryDefault();
    leftDrive4Follower.configFactoryDefault();
    
    /* setup followers */
    rightDrive2Follower.follow(rightDrive1);
    leftDrive4Follower.follow(leftDrive3);


    /** Intake setup is restore needed? */
    rightIntake.restoreFactoryDefaults();
    leftIntake.restoreFactoryDefaults();
    climber.restoreFactoryDefaults();
    roller.configFactoryDefault();

    /** rightIntake motor is mounted opposite of leftIntake motor */
    rightIntake.setInverted(true);
    leftIntake.setInverted(false);
    roller.setInverted(false); // check this out

    // rightIntake.setSmartCurrentLimit(30);
    // leftIntake.setSmartCurrentLimit(30);
    /**
    * Uses the CameraServer class to automatically capture video from a USB webcam and send it to the
    * FRC dashboard without doing any vision processing. This is the easiest way to get camera images
    * to the dashboard. Just add this to the robotInit() method in your program.
    */

    CameraServer.startAutomaticCapture(0); // Intake Camera
    CameraServer.startAutomaticCapture(1); // Shooter Camera
  }

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
    autonCounter = 0;
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    
   /**  Run conveyer for 3 seconds */
  if (autonCounter < 90) {
    rightIntake.set(FULLSPEED);
    leftIntake.set(FULLSPEED);
  }
  else {
    rightIntake.stopMotor();
    leftIntake.stopMotor();
  }
   
 /** try going 12 feet forward */
 //if (autonTimer.get()<10.0)&&(autonTimer.get()>3))
 if ((autonCounter>90)&&(autonCounter<310)) {
    robotDrive.arcadeDrive(0.0, 0.5); // drive forward half speed
  } else {
    robotDrive.stopMotor(); // stop robot
  }
if((autonCounter>200)&& (autonCounter<205)){
  robotDrive.arcadeDrive(0.0, 0.7);
  }
if ((autonCounter>205)&&(autonCounter<210)) {
  robotDrive.arcadeDrive(0.0, -0.7);
 }

if ((autonCounter>290)&&(autonCounter<310)) {
  roller.set(HALFSPEED);
} else{ 
roller.set(0);

}

  if ((autonCounter > 310)&&(autonCounter < 320)) {
    rightIntake.set(FULLSPEED);
    leftIntake.set(FULLSPEED);
  }
  else { if(autonCounter >90){
    rightIntake.stopMotor();
    leftIntake.stopMotor();
    }
  }
    /** Set intake as front */
    rightDrive1.setInverted(false);
    leftDrive3.setInverted(false);

    /** Invert the followers to match lead controllers */
    rightDrive2Follower.setInverted(InvertType.FollowMaster);
    leftDrive4Follower.setInverted(InvertType.FollowMaster);
  autonCounter = autonCounter +1;
  }

  /** This function is called once each time the robot enters teleoperated mode. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {
    if (driverController.getBButtonPressed()) {
      if(driveToggle) {
        
        //rightIntake.setInverted(false);
        driveToggle = false;
      } else
      {
        
        //rightIntake.setInverted(true);
        driveToggle = true;
      }
    }
    //robotDrive.arcadeDrive(driverController.getLeftX(), -driverController.getRightY());
    
    double speed = driveLimiter.calculate(driverController.getLeftX()*.7);
    double turn = turnLimter.calculate(driverController.getRightY());

    if(driveToggle){
      robotDrive.arcadeDrive(speed, -turn);

    } else{
      robotDrive.arcadeDrive(-speed, turn);
    }

    roller.set(driverController.getLeftTriggerAxis()*.5);
    if (driverController.getLeftBumperPressed()){
      if(rollerToggle) {
        roller.setInverted(true);
        rollerToggle = false;
      } else
      {
        roller.setInverted(false);
        rollerToggle = true;
      }
    }

    rightIntake.set(operatorController.getRightTriggerAxis()); 
    leftIntake.set(operatorController.getLeftTriggerAxis());
    
    if (operatorController.getYButton()) {climberSpeed = HALFSPEED;} 
      else {if (operatorController.getAButton()) {climberSpeed = -HALFSPEED;}
      else {climberSpeed  = 0;}
    }

    climber.set(climberSpeed);
    //rightIntake.set(operatorController.getRightTriggerAxis());
    //leftIntake.set(operatorController.getLeftTriggerAxis());

    if (operatorController.getRightBumperPressed()) {
      if(rightToggle) {
        rightIntake.setInverted(false);
        rightToggle = false;
      } else
      {
        rightIntake.setInverted(true);
        rightToggle = true;
      }
    }
    if (operatorController.getLeftBumperPressed()){
      if(leftToggle) {
        leftIntake.setInverted(true);
        leftToggle = false;
      } else
      {
        leftIntake.setInverted(false);
        leftToggle = true;
      }
    }
  }

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override

  public void testPeriodic() {}
}
