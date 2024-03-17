// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.LaunchNote;
import frc.robot.commands.PrepareLaunch;
import frc.robot.subsystems.CANDrivetrain;
import frc.robot.subsystems.CANLauncher;
import frc.robot.subsystems.Climber;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog.State;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix.led.ColorFlowAnimation.Direction;

import static edu.wpi.first.units.MutableMeasure.mutable;

//import frc.robot.subsystems.SystemIdLog;
import frc.robot.subsystems.CANIntake;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems are defined here.
  private final CANDrivetrain m_drivetrain = new CANDrivetrain();
  private final CANLauncher m_launcher = new CANLauncher();
  private final Climber m_climber = new Climber();
  private final CANIntake m_intake = new CANIntake();
  //private final SystemIdLog m_systemIdLog = new SystemIdLog();   //may break

  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));

  /*The gamepad provided in the KOP shows up like an XBox controller if the mode switch is set to X mode using the
   * switch on the top.*/
  private final CommandPS5Controller m_driverController =
      new CommandPS5Controller(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController =
      new CommandXboxController(OperatorConstants.kOperatorControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be accessed via the
   * named factory methods in the Command* classes in edu.wpi.first.wpilibj2.command.button (shown
   * below) or via the Trigger constructor for arbitary conditions
   */
  private void configureBindings() {
    // Set the default command for the drivetrain to drive using the joysticks
    m_drivetrain.setDefaultCommand(
        new RunCommand(
            () ->
                m_drivetrain.tankDrive(
                    -m_driverController.getLeftY(), -m_driverController.getRightY()),
            m_drivetrain));
            
    
    m_climber.setDefaultCommand(
        new RunCommand(
            () -> 
                m_climber.stickControl(
                    -m_operatorController.getLeftY(), -m_operatorController.getRightY()), 
            m_climber));

    /*Create an inline sequence to run when the operator presses and holds the A (green) button. Run the PrepareLaunch
     * command for 1 seconds and then run the LaunchNote command */
    m_operatorController
        .a()
        .whileTrue(
            new PrepareLaunch(m_launcher));

    m_operatorController.leftTrigger().whileTrue(m_intake.getRollersInCommand());

    // Set up a binding to run the intake command while the operator is pressing and holding the
    // left Bumper
    m_operatorController.leftBumper().whileTrue(m_launcher.getIntakeCommand());
    
    // new sequence to luanch notes
    m_operatorController.a()
    .and(m_operatorController.rightBumper())
        .whileTrue(new LaunchNote(m_launcher));

    m_operatorController.a().onFalse(m_launcher.getStopCommand());
  }

  //SysId for drivetrain
  SysIdRoutine routine = new SysIdRoutine(
    new SysIdRoutine.Config(), 
    new SysIdRoutine.Mechanism(
      m_drivetrain::voltTankDriveMeasure, 
      log -> {
        log.motor("leftFront")
          .voltage(m_appliedVoltage.mut_replace(m_drivetrain.leftFront.get()*RobotController.getBatteryVoltage(), Volts))
          .linearPosition(m_distance.mut_replace(m_drivetrain.getLeftPositionMeters(), Meters))
          .linearVelocity(m_velocity.mut_replace(m_drivetrain.getLeftVelocityMPS(), MetersPerSecond));

        log.motor("rightFront")
          .voltage(m_appliedVoltage.mut_replace(m_drivetrain.rightFront.get()*RobotController.getBatteryVoltage(), Volts))
          .linearPosition(m_distance.mut_replace(m_drivetrain.getRightPositionMeters(), Meters))
          .linearVelocity(m_velocity.mut_replace(m_drivetrain.getRightVelocityMPS(), MetersPerSecond));
      }, 
      m_drivetrain
    )
  );

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return routine.quasistatic(direction);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    //return Autos.BlindSideAuto(m_drivetrain, m_launcher);
    //return sysIdQuasistatic(SysIdRoutine.Direction.kForward);
    return new SequentialCommandGroup(sysIdQuasistatic(SysIdRoutine.Direction.kForward), sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
  }
}
