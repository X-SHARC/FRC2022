// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  WPI_TalonSRX intakeMotor = new WPI_TalonSRX(Constants.INTAKE_ID);
  public Intake() {
    intakeMotor.setInverted(false);
  }

  public void take_ball(){
    intakeMotor.set(ControlMode.PercentOutput, 0.85);
  }

  public void ball_out(){
    intakeMotor.set(ControlMode.PercentOutput, -0.85);
  }

  public void stop(){
    intakeMotor.set(0.0);
  }
}
