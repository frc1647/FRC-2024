package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder;

import static frc.robot.Constants.IntakeConstants.kIntake_motor1ID;
import static frc.robot.Constants.IntakeConstants.kIntake_motor2ID;
import static frc.robot.Constants.IntakeConstants.kIntakeSpeed;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class intake extends SubsystemBase{
    CANSparkMax motor1, motor2;
    public intake(){
        motor1 = new CANSparkMax(kIntake_motor1ID, MotorType.kBrushed);
        motor2 = new CANSparkMax(kIntake_motor2ID, MotorType.kBrushed);
        motor1.follow(motor2);
        RelativeEncoder m_motor1 = motor1.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);//changable
        RelativeEncoder m_motor2 = motor2.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);//changable
        motor1.follow(motor2);
    }
    public Command down(){
        return startEnd(
        // When the command is initialized, set the wheels to the intake speed values
        () -> {
            set_speed(kIntakeSpeed);        },
        // When the command stops, stop the wheels
        () -> {
          stop();
        });
    }
    public void set_speed(double speed){
        motor1.set(speed);
    }
    public void stop(){
        motor1.set(0);
    }
    //public //https://docs.revrobotics.com/through-bore-encoder/specifications, to view the number of rotations, find example code for reference
    //2048 rev
    
}
