// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CANDrivetrain;
import frc.robot.subsystems.CANLauncher;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static Command exampleAuto(CANDrivetrain drivetrain) {
    /**
     * RunCommand is a helper class that creates a command from a single method, in this case we
     * pass it the arcadeDrive method to drive straight back at half power. We modify that command
     * with the .withTimeout(1) decorator to timeout after 1 second, and use the .andThen decorator
     * to stop the drivetrain after the first command times out
     */
    return new RunCommand(() -> drivetrain.tankDrive(.3, .3))
        .withTimeout(1)
        .andThen(new RunCommand(() -> drivetrain.tankDrive(0, 0)));
    
  }

  public static Command BlindCenterAuto(CANDrivetrain drivetrain, CANLauncher m_launcher){
    return new SequentialCommandGroup(
      new DriveStraight(drivetrain, -.32),
      new PrepareLaunch(m_launcher).withTimeout(2),
      new LaunchNote(m_launcher).withTimeout(.5),
      new DriveStraight(drivetrain, -.4)
    );
    
    /*DriveStraight(drivetrain, -.32).andThen(new PrepareLaunch(m_launcher).withTimeout(2)
      .andThen(new LaunchNote(m_launcher).withTimeout(.5)
      .andThen(m_launcher.getStopCommand().andThen(new DriveStraight(drivetrain, -.2)))));*/
  }
  
  public static Command BlindSideAuto(CANDrivetrain drivetrain, CANLauncher m_launcher){
    return new SequentialCommandGroup(
      //new DriveStraight(drivetrain, -.5),
      new PrepareLaunch(m_launcher).withTimeout(2),
      new LaunchNote(m_launcher).withTimeout(.5),
      new DriveStraight(drivetrain, -1)
    );
  }

  public static Command SMRTCenterAuto(CANDrivetrain drivetrain, CANLauncher launcher){
    return new CenterAutoCommand(drivetrain, launcher);
  }
  
  public static Command LeftSideRedAuto(CANDrivetrain drivetrain, CANLauncher launcher){
    return new LeftSideRedCommand(drivetrain, launcher);
  }

  public static Command RightSideBlueAuto(CANDrivetrain drivetrain, CANLauncher launcher){
    return new RightSideBlueCommand(drivetrain, launcher);
  }

  public static Command LeftSideTestingAuto(CANDrivetrain drivetrain, CANLauncher launcher){
    return new LeftSideTestingCommand(drivetrain, launcher);
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
