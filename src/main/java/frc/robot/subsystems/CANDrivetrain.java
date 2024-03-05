// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.DrivetrainConstants.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//NavX
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//Odometry
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;

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
  DifferentialDrive m_drivetrain;
  RelativeEncoder m_rightEncoder, m_leftEncoder;
  double rightPosition, leftPosition;
  DifferentialDriveOdometry tankOdometry;

  /*Constructor. This method is called when an instance of the class is created. This should generally be used to set up
   * member variables and perform any configuration or set up necessary on hardware.
   */
  public CANDrivetrain() {
    CANSparkMax leftFront = new CANSparkMax(kLeftFrontID, MotorType.kBrushless);
    CANSparkMax leftRear = new CANSparkMax(kLeftRearID, MotorType.kBrushless);
    CANSparkMax rightFront = new CANSparkMax(kRightFrontID, MotorType.kBrushless);
    CANSparkMax rightRear = new CANSparkMax(kRightRearID, MotorType.kBrushless);

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
    m_rightEncoder = rightFront.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
    m_leftEncoder = leftFront.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);

    // Put the front motors into the differential drive object. This will control all 4 motors with
    // the rears set to follow the fronts
    m_drivetrain = new DifferentialDrive(leftFront, rightFront);
    m_drivetrain.setDeadband(kDriveDeadband);

    //Odometry

  }

  @Override
  public void periodic() {
    leftPosition = m_leftEncoder.getPosition();
    rightPosition = m_rightEncoder.getPosition();
  }

  public void tankDrive(double leftSpeed, double rightSpeed) {
    m_drivetrain.tankDrive(.533 * Math.pow(leftSpeed,3) + .467 * leftSpeed, .533 * Math.pow(rightSpeed,3) + .467 * rightSpeed);
    //m_drivetrain.tankDrive(-0.8 * Math.pow(leftSpeed,3) + 1.8 * leftSpeed, -0.8* Math.pow(rightSpeed,3) + 1.8 * rightSpeed);
    //m_drivetrain.tankDrive(.5 * leftSpeed, .5 * rightSpeed);
  }

  public void rawTankDrive(double leftSpeed, double rightSpeed) {
    m_drivetrain.tankDrive(leftSpeed, rightSpeed);
  }

  public double getRightPosition() {
      return rightPosition;
  }

  public double getLeftPosition() {
      return leftPosition;
  }

  public Pose2d getPose(){
    return tankOdometry.getPoseMeters();
  }
}
