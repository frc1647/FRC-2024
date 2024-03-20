package frc.robot.commands;

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
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import static frc.robot.Constants.AutoConstants.RamseteConstants.*;
import frc.robot.subsystems.CANDrivetrain;
import frc.robot.subsystems.CANLauncher;

public class CenterAutoCommand extends SequentialCommandGroup{

    Trajectory trajectory1, trajectory2, trajectory3, trajectory4;

    public CenterAutoCommand(CANDrivetrain m_drivetrain, CANLauncher m_launcher){
        //gets rid of gyroscope and disables encoders
        m_drivetrain.zeroEncoders();
        m_drivetrain.resetGyro();

        /*var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                ksVolts,
                kvVoltSecondsPerMeter,
                kaVoltSecondsSquaredPerMeter),
            kDriveKinematics,
            10);*/

        // Trajectory 1
        Pose2d startingPoint = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
        Pose2d ShootPoint = new Pose2d(-.39, 0, Rotation2d.fromDegrees(0));

        var interiorWaypoints1 = new ArrayList<Translation2d>();
        interiorWaypoints1.add(new Translation2d(-.2, 0));


        // Trajectory 2
        Pose2d EndPoint = new Pose2d(-1.2, 0, Rotation2d.fromDegrees(0));

        var interiorWaypoints2 = new ArrayList<Translation2d>();
        interiorWaypoints2.add(new Translation2d(-.8, 0));

        TrajectoryConfig config = new TrajectoryConfig(1, 1);
        config.setReversed(true);

        //Trajectory 3
        Pose2d startPoint2 = new Pose2d(0.65,6.43, Rotation2d.fromDegrees(0));
        var interiorWayPoint3 = new ArrayList<Translation2d>();
        interiorWayPoint3.add(new Translation2d(2.34,7.00));
        Pose2d endPoint2 = new Pose2d(0.65,6.43, Rotation2d.fromDegrees(0));

        //Trajectory 4
        Pose2d startPoint3 = new Pose2d(0.46,4.58, Rotation2d.fromDegrees(0));
        var interiorWayPoint4 = new ArrayList<Translation2d>();
        interiorWayPoint3.add(new Translation2d(2.31,4.08));
        Pose2d endPoint3 = new Pose2d(0.46,4.58, Rotation2d.fromDegrees(0));

        trajectory1 = TrajectoryGenerator.generateTrajectory(startingPoint, interiorWaypoints1, ShootPoint, config);
        trajectory2 = TrajectoryGenerator.generateTrajectory(ShootPoint, interiorWaypoints2, EndPoint, config);
        trajectory3 = TrajectoryGenerator.generateTrajectory(startPoint2, interiorWayPoint3, endPoint2, config);
        trajectory3 = TrajectoryGenerator.generateTrajectory(startPoint3, interiorWayPoint4, endPoint3, config);

        RamseteController controller = new RamseteController();

        addCommands(
            new PrepareLaunch(m_launcher).withTimeout(2).alongWith(
            new RamseteCommand(trajectory1, () -> m_drivetrain.getPose(), controller, new SimpleMotorFeedforward(ksVolts, kvVoltSecondsPerMeter, kaVoltSecondsSquaredPerMeter), m_drivetrain.getKinematics(), m_drivetrain::getWheelSpeeds, new PIDController(kPDriveVel, 0, 0), new PIDController(kPDriveVel, 0, 0), m_drivetrain::voltTankDrive, m_drivetrain)),
            new LaunchNote(m_launcher).withTimeout(.5),
            m_launcher.getStopCommand(),
            new RamseteCommand(trajectory2, () -> m_drivetrain.getPose(), controller, new SimpleMotorFeedforward(ksVolts, kvVoltSecondsPerMeter, kaVoltSecondsSquaredPerMeter), m_drivetrain.getKinematics(), m_drivetrain::getWheelSpeeds, new PIDController(kPDriveVel, 0, 0), new PIDController(kPDriveVel, 0, 0), m_drivetrain::voltTankDrive, m_drivetrain)
        );
        
    }
}
