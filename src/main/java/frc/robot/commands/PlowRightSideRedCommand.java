
package frc.robot.commands;

import static frc.robot.Constants.AutoConstants.RamseteConstants.kPDriveVel;
import static frc.robot.Constants.AutoConstants.RamseteConstants.kaVoltSecondsSquaredPerMeter;
import static frc.robot.Constants.AutoConstants.RamseteConstants.ksVolts;
import static frc.robot.Constants.AutoConstants.RamseteConstants.kvVoltSecondsPerMeter;

import java.util.ArrayList;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.CANDrivetrain;
import frc.robot.subsystems.CANLauncher;

public class PlowRightSideRedCommand extends SequentialCommandGroup {
    Trajectory trajectory1, trajectory2, trajectory3;
    public PlowRightSideRedCommand(CANDrivetrain m_drivetrain, CANLauncher m_launcher){
        //resets gyroscope and encoders
        m_drivetrain.zeroEncoders();
        m_drivetrain.resetGyro();
        TrajectoryConfig config = new TrajectoryConfig(1.5, 1);
        config.setReversed(true);
        TrajectoryConfig config2 = new TrajectoryConfig(1.5, 1);
        config2.setReversed(false);

        //trajectory 1
        Pose2d startingPoint = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
        Pose2d ShootPoint = new Pose2d(-.05,0, Rotation2d.fromDegrees(0));

        var interiorWaypoints1 =  new ArrayList<Translation2d>();
        interiorWaypoints1.add(new Translation2d(-.025, 0));
        
        //trajectory 2
        Pose2d turningPoint = new Pose2d(.4,-1.27, Rotation2d.fromDegrees(-22.476));
        var interiorWaypoints2 = new ArrayList<Translation2d>();
        interiorWaypoints2.add(new Translation2d(-.5,.2));

        //trajectory 3
        Pose2d startingPoint3 = new Pose2d(.4, -1.27, Rotation2d.fromDegrees(0));
        Pose2d endpoint = new Pose2d(-2.3,4, Rotation2d.fromDegrees(0));
        
        var interiorWaypoints3 = new ArrayList<Translation2d>();
        interiorWaypoints3.add(new Translation2d(-1.3,-.3));

        trajectory1 = TrajectoryGenerator.generateTrajectory(startingPoint, interiorWaypoints1, ShootPoint, config);
        trajectory2 = TrajectoryGenerator.generateTrajectory(ShootPoint, interiorWaypoints2, turningPoint, config);
        trajectory3 = TrajectoryGenerator.generateTrajectory(startingPoint3, interiorWaypoints3, endpoint, config2);
        
        RamseteController controller = new RamseteController();

        addCommands(
            new PrepareLaunch(m_launcher).raceWith(new RamseteCommand(trajectory1, () -> m_drivetrain.getPose(), controller, new SimpleMotorFeedforward(ksVolts, kvVoltSecondsPerMeter, kaVoltSecondsSquaredPerMeter), m_drivetrain.getKinematics(), m_drivetrain::getWheelSpeeds, new PIDController(kPDriveVel, 0, 0), new PIDController(kPDriveVel, 0, 0), m_drivetrain::voltTankDrive, m_drivetrain)),
            new WaitCommand(1.7),
            new LaunchNote(m_launcher).withTimeout(.5),
            new RamseteCommand(trajectory2, () -> m_drivetrain.getPose(), controller, new SimpleMotorFeedforward(ksVolts, kvVoltSecondsPerMeter, kaVoltSecondsSquaredPerMeter), m_drivetrain.getKinematics(), m_drivetrain::getWheelSpeeds, new PIDController(kPDriveVel, 0, 0), new PIDController(kPDriveVel, 0, 0), m_drivetrain::voltTankDrive, m_drivetrain),
            new WaitCommand(.5),
            new RamseteCommand(trajectory3, () -> m_drivetrain.getPose(), controller, new SimpleMotorFeedforward(ksVolts, kvVoltSecondsPerMeter, kaVoltSecondsSquaredPerMeter), m_drivetrain.getKinematics(), m_drivetrain::getWheelSpeeds, new PIDController(kPDriveVel, 0, 0), new PIDController(kPDriveVel, 0, 0), m_drivetrain::voltTankDrive, m_drivetrain)
        );
    }
}