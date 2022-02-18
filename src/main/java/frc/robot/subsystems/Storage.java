// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Storage extends SubsystemBase {
  WPI_TalonSRX storageMotor = new WPI_TalonSRX(Constants.STORAGE_ID);
  public Storage() {
    storageMotor.setInverted(true);
  }

  public void storageForward(){
    storageMotor.set(ControlMode.PercentOutput, 0.7);
  }

  public void storageBackwards(){
    storageMotor.set(ControlMode.PercentOutput, -0.7);
  }

  public void stop(){
    storageMotor.set(ControlMode.PercentOutput, 0.0);
  }
}
