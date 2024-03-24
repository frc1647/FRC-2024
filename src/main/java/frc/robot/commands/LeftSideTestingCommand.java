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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.CANDrivetrain;
import frc.robot.subsystems.CANLauncher;

public class LeftSideTestingCommand extends SequentialCommandGroup{

    Trajectory trajectory1, trajectory2, trajectory3;

    public LeftSideTestingCommand(CANDrivetrain m_drivetrain, CANLauncher m_launcher){
        //gets rid of gyroscope and disables encoders
        m_drivetrain.zeroEncoders();
        m_drivetrain.resetGyro();
        TrajectoryConfig config = new TrajectoryConfig(1.5, 1);
        config.setReversed(true);

        //trajectory 1
        Pose2d startingPoint = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
        //Pose2d //ShootPoint = new Pose2d(SmartDashboard.getNumber("X Teasting Value", -0.09), SmartDashboard.getNumber("Y Teasting Value", .03), Rotation2d.fromDegrees(SmartDashboard.getNumber("Degrees Teasting Value", -5)));
        Pose2d ShootPoint = new Pose2d(-0.01, 0, Rotation2d.fromDegrees(3.13));

        var interiorWaypoints1 = new ArrayList<Translation2d>();
        interiorWaypoints1.add(new Translation2d(0.004,0));

        trajectory1 = TrajectoryGenerator.generateTrajectory(startingPoint, interiorWaypoints1, ShootPoint, config);

        RamseteController controller = new RamseteController();

        addCommands(
            new PrepareLaunch(m_launcher).raceWith(
            new RamseteCommand(trajectory1, () -> m_drivetrain.getPose(), controller, new SimpleMotorFeedforward(ksVolts, kvVoltSecondsPerMeter, kaVoltSecondsSquaredPerMeter), m_drivetrain.getKinematics(), m_drivetrain::getWheelSpeeds, new PIDController(kPDriveVel, 0, 0), new PIDController(kPDriveVel, 0, 0), m_drivetrain::voltTankDrive, m_drivetrain)),
            new WaitCommand(1.8),
            new LaunchNote(m_launcher).withTimeout(.5)
        );

    }
}