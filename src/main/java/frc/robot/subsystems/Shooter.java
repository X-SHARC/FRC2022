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
import edu.wpi.first.wpilibj.RobotController;
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

  private double kS = 0.74791;
  private double kV = 0.11447;
  private double kA = 0.0063222;

  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV, kA);

  private double shooterCurrentRPM;
  private double PIDOutput;
  private double feedForwardOutput;
  
  public PIDController shooterPID = new PIDController(kP, kI, kD);  

  double error;
  double output;

  public Shooter() {
    shooterSlaveMotor.setInverted(false);
    shooterMasterMotor.setInverted(true);
    shooterSlaveMotor.follow(shooterMasterMotor);
  }

  public void setRPM(int RPM){
    double shooterRawSensor = shooterMasterMotor.getSelectedSensorVelocity();
    shooterCurrentRPM = (shooterRawSensor * 10.)/2048. * 60.;
    //2048 signals per revolution

    shooterPID.setTolerance(100);
    PIDOutput = shooterPID.calculate(shooterCurrentRPM/60., RPM/60.);

    feedForwardOutput = feedforward.calculate(RPM/60.);

    output = (PIDOutput + feedForwardOutput) / RobotController.getBatteryVoltage();
    shooterMasterMotor.set(ControlMode.PercentOutput,output);
  }

  public double getRPM(){
    return shooterCurrentRPM;
  }

  public void shootBall(){
    shooterMasterMotor.set(ControlMode.PercentOutput, 0.52);
  }

  public void stop(){
    shooterMasterMotor.set(ControlMode.PercentOutput, 0);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter RPM", shooterCurrentRPM);
    // SmartDashboard.putNumber("Shooter FeedForward Output", feedForwardOutput );
    // SmartDashboard.putNumber("Shooter PID Output", output);
    // SmartDashboard.putNumber("Shooter Setpoint", shooterPID.getSetpoint());

    
  }
}
