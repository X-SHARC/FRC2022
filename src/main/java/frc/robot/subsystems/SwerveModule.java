// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;
import frc.robot.lib.util.Gearbox;

public class SwerveModule {
  // CANCoder & SRXMagEncoder has 4096 ticks/rotation
  private static double kEncoderTicksPerRotation = 4096;

  private String name;
  private Rotation2d offset;
  private TalonFX driveMotor;
  private TalonFX angleMotor;
  //private Encoder rotEncoder;
  private DutyCycleEncoder rotEncoder;
  // DutyCycleEncoder is used for absolute values. Switch to normal Encoder class for relative.
  // Using absolute has the advantage of zeroing the modules autonomously.
  // If using relative, find a way to mechanically zero out wheel headings before starting the robot.
  Gearbox driveRatio = new Gearbox(6.86, 1);
  
  private PIDController rotPID = new PIDController(Constants.Swerve.kAngleP, 0, 0);

  private PIDController drivePID = new PIDController(Constants.Swerve.kDriveP, Constants.Swerve.kDriveI, Constants.Swerve.kDriveD);

  private final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(Constants.Swerve.kDriveS, Constants.Swerve.kDriveV, Constants.Swerve.kDriveA);

  private int resetOffset = 0;

  public SwerveModule(String name, TalonFX driveMotor, TalonFX angleMotor, DutyCycleEncoder rotEncoder, Rotation2d offset) {
    this.name = name;
    this.driveMotor = driveMotor;
    this.angleMotor = angleMotor;
    this.rotEncoder = rotEncoder;
    this.offset = offset;
    this.driveMotor.setNeutralMode(NeutralMode.Brake);
    this.angleMotor.setNeutralMode(NeutralMode.Brake);
    rotPID.disableContinuousInput();
  }

  public double getDegrees(){
    return Math.IEEEremainder((rotEncoder.get() * 360. + offset.getDegrees()),
     360.);
  }

  public double getPosition(){
    return driveMotor.getSelectedSensorPosition() / 2048.0 * Constants.Swerve.wheelCircumference;
  }

    // ! added drive ratio, check odometry
  public double getDriveMotorRate(){
    return driveRatio.calculate(
      ((driveMotor.getSelectedSensorVelocity() * 10) / 2048.0) * Constants.Swerve.wheelCircumference
    );
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(
      getDriveMotorRate(), 
      getAngle()
      );
  }

  public Rotation2d getAngle() {
    return Rotation2d.fromDegrees(
     getDegrees()
    );
  }

  public void calibrate(String Name, boolean offsetCalibration, boolean driveCalibration, boolean rotCalibration){
    if(offsetCalibration){
      SmartDashboard.putNumber(Name + " Rot Encoder Value", getAngle().getDegrees());
      SmartDashboard.putNumber(Name + " PID Setpoint", rotPID.getSetpoint());
    }
    // ? all the values below should be tunable in Glass
    if(rotCalibration){
      SmartDashboard.putData(Name + " Rotation PID", rotPID);     
    }
    if(driveCalibration){
      SmartDashboard.putData(Name + " Drive PID", drivePID);
    }
  }

  public void resetRotationEncoder(){
    rotEncoder.reset();
  }
  
  public void resetDriveEncoder(){
    driveMotor.setSelectedSensorPosition(0);    
  }

  public void resetDriveEncoderbyOffset(){
    //implement resetting by offset
  }

  public void resetBothEncoders(){
    resetDriveEncoder();
    resetRotationEncoder();
  }

  public void stopMotors(){
    driveMotor.set(TalonFXControlMode.PercentOutput, 0);
    angleMotor.set(TalonFXControlMode.PercentOutput, 0);
  }

  public void setClosedLoop(SwerveModuleState desiredState){
    Rotation2d currentRotation = getAngle();
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, currentRotation);
    // Find the difference between our current rotational position + our new rotational position
    Rotation2d rotationDelta = state.angle.minus(currentRotation);
    double desiredRotation = currentRotation.getDegrees() + rotationDelta.getDegrees();

    angleMotor.set(TalonFXControlMode.PercentOutput, 
        MathUtil.clamp( 
          ( rotPID.calculate(
                currentRotation.getDegrees(),
                desiredRotation 
                )), 
            -1.0, 
            1.0)
    );

    double driveOutput = driveFeedforward.calculate(state.speedMetersPerSecond) + drivePID.calculate(getDriveMotorRate(), state.speedMetersPerSecond);
    driveOutput = driveOutput / RobotController.getBatteryVoltage();
    driveMotor.set(TalonFXControlMode.PercentOutput, driveOutput);
    
  }

  // ! Open loop drive
  public void setDesiredState(SwerveModuleState desiredState) {
    if(Math.abs(desiredState.speedMetersPerSecond)<0.001){
      stopMotors();
      return;
    }

    Rotation2d currentRotation = getAngle();
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, currentRotation);
    // Find the difference between our current rotational position + our new rotational position
    Rotation2d rotationDelta = state.angle.minus(currentRotation);
    double desiredRotation = currentRotation.getDegrees() + rotationDelta.getDegrees();

    angleMotor.set(TalonFXControlMode.PercentOutput, 
        MathUtil.clamp( 
          ( rotPID.calculate(
                currentRotation.getDegrees(),
                desiredRotation 
                )), 
            -1.0, 
            1.0)
    );

    driveMotor.set(TalonFXControlMode.PercentOutput, state.speedMetersPerSecond / Constants.Swerve.kMaxSpeed);
  }

}