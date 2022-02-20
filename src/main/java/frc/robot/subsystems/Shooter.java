// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  WPI_TalonFX shooterMasterMotor = new WPI_TalonFX(Constants.SHOOTER_MASTER_ID);
  WPI_TalonFX shooterSlaveMotor = new WPI_TalonFX(Constants.SHOOTER_SLAVE_ID);

  private double kP = 0.11533;
  private double kI = 0.0;
  private double kD = 0.0;

  private double kS = 0.11447;
  private double kV = 0.11447;
  private double kA = 0.0063222;

  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV, kA);

  private double shooterCurrentRPM;
  private double PIDOutput;
  private double feedForwardOutput;
  
  PIDController shooterPID = new PIDController(kP, kI, kD);  

  int RPM;
  double error;
  double output;

  public Shooter() {
    shooterSlaveMotor.setInverted(false);
    shooterMasterMotor.setInverted(true);
    shooterSlaveMotor.follow(shooterMasterMotor);
  }

  public void pidShooter(int RPM){
    double shooterRawSensor = shooterMasterMotor.getSelectedSensorVelocity();

    this.RPM = RPM;

    shooterRawSensor *= 10;
    shooterRawSensor/=2048;
    shooterCurrentRPM = shooterRawSensor*60;
    //2048 signals per revolution

    shooterPID.setTolerance(200);
    PIDOutput = shooterPID.calculate(shooterCurrentRPM, RPM);

    feedForwardOutput = feedforward.calculate(RPM);

    error = RPM - shooterCurrentRPM;
    error/=100;
    output = error * kP/10;
    shooterMasterMotor.set(ControlMode.PercentOutput,output);
    //shooterMasterMotor.set(ControlMode.PercentOutput,PIDOutput);
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
    SmartDashboard.putNumber("Shooter RPM", shooterCurrentRPM);
    SmartDashboard.putNumber("Shooter FeedForward Output", feedForwardOutput );
    SmartDashboard.putNumber("Shooter Setpoint", RPM);
    SmartDashboard.putNumber("Shooter PID Output", PIDOutput);
    SmartDashboard.putNumber("Shooter handmade PID output", output);

    
  }
}
