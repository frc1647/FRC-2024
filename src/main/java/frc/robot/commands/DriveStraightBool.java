package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANDrivetrain;
import frc.robot.subsystems.CANLauncher;

public class DriveStraightBool extends Command{

    double position = 0;
    double goal, error, output;
    CANDrivetrain m_drivetrain;
    CANLauncher m_Launcher;
    boolean forward;

    public DriveStraightBool(CANDrivetrain drivetrain, double goalMeters, boolean localForward){
        goal = Math.abs(goalMeters);
        m_drivetrain = drivetrain;
        forward = localForward;
        addRequirements(m_drivetrain);
    }

    @Override
    public void initialize(){
        m_drivetrain.zeroEncoders();
    }

    @Override
    public void execute(){
        
        position = m_drivetrain.getLeftPositionMeters();
        if(forward){
            error = goal - position;
            output = .4 * error + .5;
        } else {
            error = goal + position;
            output = (-.4)*error - .5;
        }

        m_drivetrain.rawTankDrive(output, output);
    }

    @Override
    public boolean isFinished() {
        if (forward && position > goal){
            return true;
        } else if (forward == false && -position > goal) {
            return true;
        } else {
            return false;
        }
    }

}
