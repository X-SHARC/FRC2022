// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  WPI_TalonFX shooterMasterMotor = new WPI_TalonFX(Constants.SHOOTER_MASTER_ID);
  WPI_TalonFX shooterSlaveMotor = new WPI_TalonFX(Constants.SHOOTER_SLAVE_ID);

  private double kP = 0.0;
  private double kI = 0.0;
  private double kD = 0.0;
  
  PIDController shooterPID = new PIDController(kP, kI, kD);  

  public Shooter() {
    shooterSlaveMotor.setInverted(true);
    shooterMasterMotor.setInverted(false);
    shooterSlaveMotor.follow(shooterMasterMotor);
  }

  public void pidShooter(int RPM){
    double shooterRawSensor = shooterMasterMotor.getSelectedSensorPosition();

    shooterRawSensor/=2048;
    double shooterCurrentRPM = shooterRawSensor*60;
    //2048 signals per rotation

    double PIDOutput = shooterPID.calculate(shooterCurrentRPM, RPM);

    shooterMasterMotor.set(ControlMode.PercentOutput,PIDOutput);
    SmartDashboard.putNumber("Shooter RPM", shooterCurrentRPM);
    SmartDashboard.putNumber("Shooter PIDOutput", PIDOutput);
  }

  public void shootBall(){
    shooterMasterMotor.set(ControlMode.PercentOutput, 0.1);
  }

  public void stop(){
    shooterMasterMotor.set(ControlMode.PercentOutput, 0);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
