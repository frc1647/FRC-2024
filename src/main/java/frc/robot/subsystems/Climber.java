package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
//import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ClimberConstants.*;

public class Climber extends SubsystemBase {
    TalonFX left;
    TalonFX right;

    public Climber() {
        left = new TalonFX(kLeftClimberID);
        right = new TalonFX(kRightClimberID);

        left.setNeutralMode(NeutralModeValue.Brake);
        right.setNeutralMode(NeutralModeValue.Brake);
    }

    // butten press functions (not used rn)
    public Command setLeftDownCommand() {
        // The startEnd helper method takes a method to call when the command is initialized and one to
        // call when it ends
        return this.startEnd(
            // When the command is initialized, set the wheels to the intake speed values
            () -> {
                left.set(kLeftDownSpeed);
            },
            // When the command stops, stop the wheels
            () -> {
              stopLeft();
            });
      }

    public Command setLeftUpCommand() {
        // The startEnd helper method takes a method to call when the command is initialized and one to
        // call when it ends
        return this.startEnd(
            // When the command is initialized, set the wheels to the intake speed values
            () -> {
                left.set(kLeftUpSpeed);
            },
            // When the command stops, stop the wheels
            () -> {
            	stopLeft();
            });
      }

    public Command setRightDownCommand() {
        // The startEnd helper method takes a method to call when the command is initialized and one to
        // call when it ends
        return this.startEnd(
            // When the command is initialized, set the wheels to the intake speed values
            () -> {
                right.set(kRightDownSpeed);
            },
            // When the command stops, stop the wheels
            () -> {
              stopRight();
            });
      }
      public Command setRightUpCommand() {
        // The startEnd helper method takes a method to call when the command is initialized and one to
        // call when it ends
        return this.startEnd(
            // When the command is initialized, set the wheels to the intake speed values
            () -> {
                right.set(kRightUpSpeed);
            },
            // When the command stops, stop the wheels
            () -> {
              stopRight();
            });
      }


      //variable speed control
      public void stickControl(double leftSpeed, double rightSpeed) {
        if (Math.abs(leftSpeed) > kDeadzone) {
          left.set(leftSpeed);
        } else {
          left.set(0);
        }
        
        if (Math.abs(rightSpeed) > kDeadzone) {
          right.set(rightSpeed);
        } else {
          right.set(0);
        }
      }


      // stop commands
      public void stopRight(){
        right.set(0);
      }
      
      public void stopLeft(){
        left.set(0);
      }
}
