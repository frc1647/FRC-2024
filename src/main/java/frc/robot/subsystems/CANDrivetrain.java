// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.DrivetrainConstants.*;

//NavX
import com.kauailabs.navx.frc.AHRS;
//motors
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/* This class declares the subsystem for the robot drivetrain if controllers are connected via CAN. Make sure to go to
 * RobotContainer and uncomment the line declaring this subsystem and comment the line for PWMDrivetrain.
 *
 * The subsystem contains the objects for the hardware contained in the mechanism and handles low level logic
 * for control. Subsystems are a mechanism that, when used in conjuction with command "Requirements", ensure
 * that hardware is only being used by 1 command at a time.
 */
public class CANDrivetrain extends SubsystemBase {
  /*Class member variables. These variables represent things the class needs to keep track of and use between
  different method calls. */
  public CANSparkMax leftFront, rightFront;
  CANSparkMax leftRear, rightRear;
  DifferentialDrive m_drivetrain;

  public RelativeEncoder m_rightEncoder, m_leftEncoder;
  double rightPositionMeters, leftPositionMeters, leftVelocityMPS, rightVelocityMPS, batteryVolts;

  AHRS navX;

  DifferentialDriveOdometry tankOdometry;
  Pose2d m_pose;

  Trajectory exampleTrajectory;

  RamseteController controller;

  DifferentialDriveKinematics tankKinematics;
  DifferentialDriveWheelSpeeds wheelSpeeds;

  /*Constructor. This method is called when an instance of the class is created. This should generally be used to set up
   * member variables and perform any configuration or set up necessary on hardware.
   */
  public CANDrivetrain() {
    leftFront = new CANSparkMax(kLeftFrontID, MotorType.kBrushless);
    leftRear = new CANSparkMax(kLeftRearID, MotorType.kBrushless);
    rightFront = new CANSparkMax(kRightFrontID, MotorType.kBrushless);
    rightRear = new CANSparkMax(kRightRearID, MotorType.kBrushless);

    /*Sets current limits for the drivetrain motors. This helps reduce the likelihood of wheel spin, reduces motor heating
     *at stall (Drivetrain pushing against something) and helps maintain battery voltage under heavy demand */
    leftFront.setOpenLoopRampRate(kRampRate);
    leftRear.setOpenLoopRampRate(kRampRate);
    rightFront.setOpenLoopRampRate(kRampRate);
    rightRear.setOpenLoopRampRate(kRampRate);

    // coast mode
    leftFront.setIdleMode(IdleMode.kCoast);
    leftRear.setIdleMode(IdleMode.kCoast);
    rightFront.setIdleMode(IdleMode.kCoast);
    rightRear.setIdleMode(IdleMode.kCoast);

    // Set the rear motors to follow the front motors.
    leftRear.follow(leftFront);
    rightRear.follow(rightFront);

    // Invert the left side so both side drive forward with positive motor outputs
    leftFront.setInverted(false);
    rightFront.setInverted(true);

    //encoders
    m_leftEncoder = leftFront.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
    m_rightEncoder = rightFront.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
    m_leftEncoder.setPositionConversionFactor(kPositionConversionFactor);
    m_rightEncoder.setPositionConversionFactor(kPositionConversionFactor);
    m_leftEncoder.setVelocityConversionFactor(kVelocityConversionFactor);
    m_rightEncoder.setVelocityConversionFactor(kVelocityConversionFactor);

    // Put the front motors into the differential drive object. This will control all 4 motors with
    // the rears set to follow the fronts
    m_drivetrain = new DifferentialDrive(leftFront, rightFront);
    m_drivetrain.setDeadband(kDriveDeadband);

    //NavX
    navX = new AHRS(SPI.Port.kMXP);

    tankOdometry = new DifferentialDriveOdometry(navX.getRotation2d(), leftPositionMeters, rightPositionMeters);

    tankKinematics = new DifferentialDriveKinematics(kDrivetrainWidthMeters);

    wheelSpeeds = new DifferentialDriveWheelSpeeds();
  }

  @Override
  public void periodic() {
    leftPositionMeters = m_leftEncoder.getPosition();
    rightPositionMeters = m_rightEncoder.getPosition();

    leftVelocityMPS = m_leftEncoder.getVelocity();
    rightVelocityMPS = m_rightEncoder.getVelocity();

    wheelSpeeds.leftMetersPerSecond = leftVelocityMPS;
    wheelSpeeds.rightMetersPerSecond = rightVelocityMPS;

    m_pose = tankOdometry.update(navX.getRotation2d(), leftPositionMeters, leftPositionMeters);

    batteryVolts = RobotController.getBatteryVoltage();   //only needed for SysID Testing

    SmartDashboard.putNumber("left Encoder", leftPositionMeters);
    SmartDashboard.putNumber("right Encoder", rightPositionMeters);
    SmartDashboard.putNumber("left Velo MPS", leftVelocityMPS);
    SmartDashboard.putNumber("right Velo MPS", rightVelocityMPS);
    }

  public void tankDrive(double leftSpeed, double rightSpeed) {
    m_drivetrain.tankDrive(.533 * Math.pow(leftSpeed,3) + .467 * leftSpeed, .533 * Math.pow(rightSpeed,3) + .467 * rightSpeed);
    //m_drivetrain.tankDrive(-0.8 * Math.pow(leftSpeed,3) + 1.8 * leftSpeed, -0.8* Math.pow(rightSpeed,3) + 1.8 * rightSpeed);
    //m_drivetrain.tankDrive(.5 * leftSpeed, .5 * rightSpeed);
  }

  public void rawTankDrive(double leftSpeed, double rightSpeed) {
    m_drivetrain.tankDrive(leftSpeed, rightSpeed);
  }

  public void voltTankDrive(double leftVolts, double rightVolts){
    leftFront.setVoltage(leftVolts);
    rightFront.setVoltage(rightVolts);
  }

  public void voltsTankDriveMeasure(Measure<Voltage> volts){       // only used for Sys Id tests
    leftFront.set(volts.magnitude() / RobotController.getBatteryVoltage());
    rightFront.set(volts.magnitude() / RobotController.getBatteryVoltage());
  }

  public double getLeftVolts(){
    return leftFront.get() * batteryVolts;
  }

  public double getRightVolts(){
    return rightFront.get() * batteryVolts;
  }

  public double getRightPositionMeters() {
      return rightPositionMeters;
  }

  public double getLeftPositionMeters() {
      return leftPositionMeters;
  }

  public double getLeftVelocityMPS(){
    return leftVelocityMPS;
  }

  public double getRightVelocityMPS(){
    return rightVelocityMPS;
  }

  public Pose2d getPose(){
    return tankOdometry.getPoseMeters();
  }

  public void zeroEncoders(){
    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);
  }

  public void resetGyro(){
    navX.reset();
  }

  public DifferentialDriveKinematics getKinematics(){
    return tankKinematics;
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds(){
    return wheelSpeeds;
  }

  public void setCoastMode(){
    leftFront.setIdleMode(IdleMode.kCoast);
    leftRear.setIdleMode(IdleMode.kCoast);
    rightFront.setIdleMode(IdleMode.kCoast);
    rightRear.setIdleMode(IdleMode.kCoast);
  }

  public void setBrakeMode(){
    leftFront.setIdleMode(IdleMode.kBrake);
    leftRear.setIdleMode(IdleMode.kBrake);
    rightFront.setIdleMode(IdleMode.kBrake);
    rightRear.setIdleMode(IdleMode.kBrake);
  }
}
