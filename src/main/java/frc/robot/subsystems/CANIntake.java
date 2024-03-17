package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import static frc.robot.Constants.IntakeConstants.*;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CANIntake extends SubsystemBase{
    CANSparkMax armMotor;
    RelativeEncoder armEncoder;
    WPI_TalonSRX rollerMotor;
    double ArmPosition;
    //Encoder throughBoreEncoder;
    DutyCycleEncoder throughBoreEncoder;

    public CANIntake(){
        armMotor = new CANSparkMax(kArmID, MotorType.kBrushed);
        rollerMotor = new WPI_TalonSRX(kRollerID);

        //throughBoreEncoder = new Encoder(0, 1, false, Encoder.EncodingType.k2X);
        throughBoreEncoder = new DutyCycleEncoder(0);

    }

    @Override
    public void periodic(){
        ArmPosition = throughBoreEncoder.getDistance();
        SmartDashboard.putNumber("Arm Encoder", ArmPosition);
    }
    
    public Command down(){      //I don't think this will work
        return startEnd(
        // When the command is initialized, set the motor to the intake speed values
        () -> {
            armVolts(kIntakeSpeed);
        },
        // When the command stops, stop the motor
        () -> {
          stopArm();
        });
    }

    public Command getRollersInCommand(){
        return startEnd(
        // When the command is initialized, set the motor to the intake speed values
        () -> {
            rollerVolts(kRollerIntakeVolts);
        },
        // When the command stops, stop the motor
        () -> {
          stopRollers();
        });
    }

    public void armVolts(double volts){
        armMotor.setVoltage(volts);
    }

    public void stopArm(){
        armMotor.set(0);
    }

    public void rollerVolts(double volts){
        rollerMotor.setVoltage(volts);
    }

    public void stopRollers(){
        rollerMotor.set(0);
    }

    //public //https://docs.revrobotics.com/through-bore-encoder/specifications, to view the number of rotations, find example code for reference
    //2048 rev
}
