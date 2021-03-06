/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private final XboxController _driverController = new XboxController(0);
  private final Drivetrain _drive = new Drivetrain();
  private final PixyController _pixycontroller = new PixyController();
  private final Shooter _shooter = new Shooter();
  private final Conveyor _conveyor = new Conveyor();
  private final Lifter _lifter = new Lifter();
  private final Compressor _compressor = new Compressor(10);

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    _drive.init();
    _pixycontroller.init();
    _shooter.init();
    _conveyor.init();
    _lifter.init();
    _compressor.setClosedLoopControl(true);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /**
   * This function is called periodically during autonomous.
   */
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

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    drivetrainLogic();
    conveyorLogic();
    shooterLogic();
    lifterLogic();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
    var ballInPosition = _conveyor.isBallInLoadingPosition();
    SmartDashboard.putBoolean("Ball In Position", ballInPosition);
    drivetrainLogic();
    conveyorLogic();
    shooterLogic();
    lifterLogic();
  }

  private void drivetrainLogic(){
    var speed = 0.0;
    var rotation = 0.0;
    var filterSpeedDeadband = false;
    var filterRotationDeadband = false;

    if(_driverController.getAButtonPressed()){
      _pixycontroller.turnLightOn(255, 255, 255);
      _pixycontroller.setBrightness(30);
    }

    if(_driverController.getXButtonPressed()){
      _pixycontroller.turnLightOn(255, 255, 255);
      _pixycontroller.setBrightness(10);
    }

    if(_driverController.getAButtonReleased() || _driverController.getXButtonReleased()){
      _pixycontroller.turnLightOff();
    }

    if(_driverController.getAButton()){
      speed = -(_driverController.getY(Hand.kLeft));
      rotation = _pixycontroller.trackBall();
      filterSpeedDeadband = true;
      filterRotationDeadband = false;
    } else if (_driverController.getXButton()) {
      speed = -(_driverController.getY(Hand.kLeft));
      rotation = _pixycontroller.trackTarget();
      filterSpeedDeadband = true;
      filterRotationDeadband = false;
    } else {
      speed = -(_driverController.getY(Hand.kLeft));
      rotation = _driverController.getX(Hand.kRight);
      filterSpeedDeadband = true;
      filterRotationDeadband = true;
    }

    _drive.arcadeDrive(speed, filterSpeedDeadband, rotation, filterRotationDeadband);
  }

  private void conveyorLogic(){
    SmartDashboard.putBoolean("Ball In Position", _conveyor.isBallInLoadingPosition());
    if (_driverController.getBumper(Hand.kRight)){
      if (_conveyor.isBallInLoadingPosition()){
        _conveyor.run(0.30);
      } else {
        _conveyor.run(0);
      }
      return;
    }

    if (_driverController.getBumper(Hand.kLeft)){
      _conveyor.run(-0.30);
    } else{
      _conveyor.run(0);
    }
  }

  private void shooterLogic(){
    var trigger = _driverController.getTriggerAxis(Hand.kRight);
    if (trigger <= 0.1){
      _shooter.fire(0);
      _shooter.closeGate();
      return;
    }

    _shooter.fire(trigger);

    if (_shooter.isAtSetPower(trigger)){
      _conveyor.run(0.30);
      _shooter.openGate();
    } else{
      _shooter.closeGate();
    }
  }

  private void lifterLogic() {
    if (_driverController.getYButton()){
      _lifter.moveHook(0.3);
    } else if (_driverController.getXButton()){
      _lifter.moveHook(-0.1);
    } else {
      _lifter.moveHook(0);
    }

    if (_driverController.getBButton()){
      _lifter.moveRobot(0.75);
    } else if (_driverController.getAButton()){
      _lifter.moveRobot(-0.75);
    } else{
      _lifter.moveRobot(0);
    }
  }
}
