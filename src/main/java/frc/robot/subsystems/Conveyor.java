package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Conveyor extends SubsystemBase {
  /** Creates a new Conveyor. */
  WPI_TalonSRX conveyorMotor = new WPI_TalonSRX(Constants.CONVEYOR_ID);
  
  public Conveyor() {
    conveyorMotor.setInverted(false);
  }

  public void feedBall(){
    conveyorMotor.set(ControlMode.PercentOutput, 0.7);
  }
  public void retractBall(){
    conveyorMotor.set(ControlMode.PercentOutput, -0.5);
  }
  public void stop(){
    conveyorMotor.set(0.0);
  }
}
