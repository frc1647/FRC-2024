package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANDrivetrain;
import frc.robot.subsystems.CANLauncher;

public class DriveStraight extends Command{

    double position = 0;
    double goal, error, output;
    CANDrivetrain m_drivetrain;
    CANLauncher m_Launcher;


    public DriveStraight(CANDrivetrain drivetrain, double goalMeters){
        goal = goalMeters;
        m_drivetrain = drivetrain;
        addRequirements(m_drivetrain);
    }

    @Override
    public void initialize(){
        m_drivetrain.zeroEncoders();
    }

    @Override
    public void execute(){
        position = m_drivetrain.getLeftPositionMeters();
        error = goal - position;
        output = .4 * error + Math.copySign(.5, error);

        m_drivetrain.rawTankDrive(output, output);

    }

    @Override
    public boolean isFinished() {
        if (goal > 0 && position > goal){
            return true;
        } if (goal < 0 && position < goal) {
            return true;
        } else {
            return false;
        }
        
        //return false;//Math.abs(position) > Math.abs(goal);
    }

}
