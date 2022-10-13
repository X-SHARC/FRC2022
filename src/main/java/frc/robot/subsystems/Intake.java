// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  WPI_TalonSRX intakeMotor = new WPI_TalonSRX(Constants.INTAKE_ID);
  public DoubleSolenoid intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM ,Constants.INTAKE_SOLENOID_FORWARD_ID, Constants.INTAKE_SOLENOID_REVERSE_ID);
  public boolean intakeExtended = false;


  public Intake() {
    intakeMotor.setInverted(false);
  }

  public void retractIntake(){
    intakeSolenoid.set(DoubleSolenoid.Value.kReverse);
    intakeExtended = false;
  }

  public void extendIntake(){
    intakeSolenoid.set(DoubleSolenoid.Value.kForward);
    intakeExtended = true;
  }

  public void runForward(){
    intakeMotor.set(ControlMode.PercentOutput, 0.88);
  }

  public void runBackwards(){
    intakeMotor.set(ControlMode.PercentOutput, -0.55);

  }

  public void stop(){
    intakeMotor.set(0.0);
  }
}
