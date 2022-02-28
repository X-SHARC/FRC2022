// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climb extends SubsystemBase {
  /** Creates a new Climb. */
  //WPI_TalonFX climbMotor = new WPI_TalonFX(Constants.CLIMB_ID);
  WPI_TalonSRX climbMotor = new WPI_TalonSRX(Constants.CLIMB_ID);

  public Climb() {
    climbMotor.setInverted(false);
    climbMotor.setNeutralMode(NeutralMode.Brake);
  }

  public void climbUp(){
    climbMotor.set(ControlMode.PercentOutput, 0.95);
  }
  public void climbDown(){
    climbMotor.set(ControlMode.PercentOutput, -0.95);
  }

  public void stop(){
    climbMotor.set(0.0);
  }
}
