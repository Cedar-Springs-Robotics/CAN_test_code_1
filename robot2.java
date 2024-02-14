// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
/*
 * 1 - drive left 1
 * 2 - drive left 2
 * 3 - drive right 1
 * 4 - drive right 2
 * 5 - shooter top
 * 6 - shooter bottom
 * 
 * 
 */




package frc.robot;

import edu.wpi.first.wpilibj.CounterBase;

import java.lang.ModuleLayer.Controller;
import java.util.HashMap;
import java.util.logging.ConsoleHandler;

import javax.xml.crypto.Data;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType; //wax added .kbrushless on 2-9-24

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.networktables.LogMessage;
// import edu.wpi.first.cscore.VideoMode.PixelFormat;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.DigitalInput;

/**
 * This is a demo program showing the use of the DifferentialDrive class. Runs
 * the motors with
 * arcade steering.
 */
public class Robot extends TimedRobot {
  private final DigitalInput m_limit = new DigitalInput(5);
  // private final PWMSparkMax m_leftMotor = new PWMSparkMax(0);
  // private final PWMSparkMax m_rightMotor = new PWMSparkMax(1); 
  private final CANSparkMax m_left1 = new CANSparkMax(1, MotorType.kBrushed);
  private final CANSparkMax m_left2 = new CANSparkMax(2, MotorType.kBrushed);
  private final CANSparkMax m_right1 = new CANSparkMax(3, MotorType.kBrushed);
  private final CANSparkMax m_right2 = new CANSparkMax(4, MotorType.kBrushed);
  private final CANSparkMax m_shooter = new CANSparkMax(6, MotorType.kBrushed);
  private final CANSparkMax m_shooter2 = new CANSparkMax(5, MotorType.kBrushless);
  private final MotorControllerGroup left = new MotorControllerGroup(m_left1, m_left2);
  private final MotorControllerGroup right = new MotorControllerGroup(m_right1, m_right2);
  private final DifferentialDrive m_robotDrive = new DifferentialDrive(right, left);
  // private final DifferentialDrive m_shoot = new DifferentialDrive(m_shooter,
  // m_shooter2);
  private final Joystick m_stick = new Joystick(0);
  // private final CommandXboxController m_controller = new
  // CommandXboxController(0);
  // private Servo servo;
  private final Encoder m_encoder = new Encoder(0, 1, false, CounterBase.EncodingType.k4X);
  private final Encoder m_encoder2 = new Encoder(2, 3, true, CounterBase.EncodingType.k4X);

  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    right.setInverted(true);
    m_shooter2.setInverted(true);
    CameraServer.startAutomaticCapture();
    DataLogManager.start();
    // servo = new Servo(2);
    // servo.set(0.5);
    m_encoder.setSamplesToAverage(5);
    m_encoder.setDistancePerPulse(1.0 / 360.0 * 2.0 * Math.PI * 3);
    m_encoder.setMinRate(1.0);

    m_encoder2.setSamplesToAverage(5);
    m_encoder2.setDistancePerPulse(1.0 / 360.0 * 2.0 * Math.PI * 3);
    m_encoder2.setMinRate(1.0);

    /*
     * UsbCamera usbCamera = new UsbCamera("USB Camera 0", 0);
     * // Inputs: Name (String), int (port)
     * MjpegServer mJPEGServer1 = new MjpegServer("serve_USB Camera 0", 1181);
     * mJPEGServer1.setSource(usbCamera);
     * 
     * // Input: Name (String)
     * CvSink cvSink = new CvSink("opencv_USB Camera 0");
     * cvSink.setSource(usbCamera);
     * 
     * // Input: Name (String), Pixel Format (enum), Resolution Width (int),
     * Resolution Height (int), FPS (int)
     * // CvSource outputStream = new CvSource("Blur", PixelFormat.kMJPEG, 160, 120,
     * 30);
     * MjpegServer mJPEGServer2 = new MjpegServer("serve_blur", 1182);
     * mJPEGServer2.setSource(outputStream);
     * // usbCamera.getInfo();
     */
  }

  private long lastPrint = 0;

  @Override
  public void robotPeriodic() {

    if (System.currentTimeMillis() - lastPrint > 1000) {
      // DataLogManager.log("Trigger Pressed: " +m_stick.getTriggerPressed());
      // DataLogManager.log("Thumb Pressed: "+m_stick.getTopPressed());
      // DataLogManager.log("Thumb check: "+m_stick.getTop());
      // DataLogManager.log("Trigger check: "+m_stick.getTrigger());
      // DataLogManager.log("Hat Moved: "+ m_stick.getPOV());
      // DataLogManager.log("Button " + m_stick.getButtonCount());
      // DataLogManager.log("Button 3 Pressed: "+ m_stick.getRawButton(3));
      DataLogManager.log("Enoder dist: " + m_encoder.getDistance());
      DataLogManager.log("Enoder dist 2: " + m_encoder2.getDistance());
      // DataLogManager.log("Servo angle: "+servo.getAngle());
      if (!m_limit.get()) {
        DataLogManager.log("Limit worked");
      }
      DataLogManager.log("");
      lastPrint = System.currentTimeMillis();
    }
  }

  // @Override
  // public void autonomousPeriodic() {
  // double xSpeed, zSpeed;

  // if (m_stick.getThrottle() == 1) {
  // xSpeed = zSpeed = 0;
  // } else {
  // xSpeed = 3 * (((-m_stick.getThrottle() + 1) / 4) + 0.5);
  // zSpeed = 0 * (((-m_stick.getThrottle() + 1) / 4) + 0.5);
  // }

  // m_robotDrive.arcadeDrive(xSpeed, zSpeed);
  // }

  double encode = 0;
  double dis = 150;
  private Long pressedJoystick = null;

  @Override
  public void teleopPeriodic() {

    /*
     * This section handles our ring shooter.
     */

    Double speed1 = null, speed2 = null;

    // If top is pressed, attempt to retract ring.
    if (m_stick.getTop()) {
      speed1 = speed2 = -0.25;
    }

    // If trigger is pressed, attempt to shoot ring.
    if (m_stick.getTrigger()) {
      long windUpTimeMS = 500;

      // Initially spin top motor.
      if (pressedJoystick == null) {
        pressedJoystick = System.currentTimeMillis();
        speed2 = 0.1;
        // Keep spinning the top motor.
      } else if ((System.currentTimeMillis() - pressedJoystick) < windUpTimeMS) {
        speed2 = 0.1;
        // After 2.5 seconds, spin the bottom motor too.
      } else {
        speed1 = 0.75;
        speed2 = 1.0;
      }
    } else {
      pressedJoystick = null;
    }

    // Set the values of the motors
    m_shooter.set(speed1 == null ? 0 : speed1);
    m_shooter2.set(speed2 == null ? 0 : speed2);

    /*
     * This section handles our general movement/driving code.
     */

    double yMove, xMove, twist = m_stick.getTwist();

    // When throttle is on -, the value is 1. Here we should not move at all.
    if (m_stick.getThrottle() == 1) {
      xMove = yMove = 0;
    } else {
      // Multiplies y/x movement by [0.5,1] depending on throttle input.
      yMove = -m_stick.getY() * (((-m_stick.getThrottle() + 1) / 4) + 0.5);
      xMove = -m_stick.getX() * (((-m_stick.getThrottle() + 1) / 4) + 0.5);
    }

    // Drive with arcade drive.
    // Y axis drives forwards and backwards, X rotates left and right.
    m_robotDrive.arcadeDrive(yMove, xMove);

    /*
     * This code handles camera rotation based on joystick twist.
     */
    // Toggleable switch for turn mode and hat mode
    double toggle = 0;

    if (m_stick.getTop()) {
      if (toggle == 1) {
        toggle = 0;
      } else {
        toggle = 1;
      }
    }

    if (toggle == 1) {

      // Prevent small accidental camera turns.

      // if (Math.abs(twist) < 0.3) twist = 0;
      // // // Servo has an input range of [0,1], but getTwist returns [-1,1]
      twist = ((int) (twist * 6)) / 6.0;
      twist = (-twist + 1) / 2;

      // servo.set(twist);
    }
    if (toggle == 0) {
      switch (m_stick.getPOV()) {
        case -1:
          // Center position
          break;
        case 0:
          // Up.
          // servo.set(0.5);
          break;
        case 45:
          // Up right
          // servo.set(0.33);
          break;
        case 90:
          // Right
          // servo.set(0.15);
          break;
        case 135:
          // Down right
          // servo.set(0.2);
          break;
        case 180:
          // Down
          // servo.set(0.5);
          break;
        case 225:
          // Down left
          // servo.set(0.8);
          break;
        case 270:
          // Left
          // servo.set(0.75);
          break;
        case 315:
          // Up left
          // servo.set(0.66);
          break;
      }
    }

    // super hidden code
    // m_robotDrive.arcadeDrive(0.5,0);
    // var time = System.currentTimeMillis();
    // while(time != System.currentTimeMillis()+1000){

    // }

    // if (System.currentTimeMillis() - lastPrint > 1000) {
    // DataLogManager.log("Twist: " + m_stick.getTwist());
    // DataLogManager.log("Throttle: " + m_stick.getThrottle());
    // DataLogManager.log("Z: " + m_stick.getZ());
    // DataLogManager.log("Y: " + m_stick.getY());
    // lastPrint = System.currentTimeMillis();
    // }
  }

  public void autonomousPeriodic() {
    // while(System.currentTimeMillis() - lastPrint > 5000){
    // m_robotDrive.arcadeDrive(0, 0);
    // lastPrint = System.currentTimeMillis();
    // }
    // while(System.currentTimeMillis() - lastPrint > 1000){
    // m_robotDrive.arcadeDrive(0.66, 0);
    // lastPrint = System.currentTimeMillis();
    // }

    // m_robotDrive.arcadeDrive(0, 0);
  }

}
