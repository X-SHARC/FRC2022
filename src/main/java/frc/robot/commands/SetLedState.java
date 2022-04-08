// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.lib.drivers.WS2812Driver;
import frc.robot.lib.drivers.WS2812Driver.Side;

public class SetLedState extends CommandBase {
  /** Creates a new SetLedState. */
  WS2812Driver led;
  public SetLedState(WS2812Driver led) {
    this.led = led;
    addRequirements(led);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    led.toggleRGB();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (RobotContainer.state.getAlignmentState()){
      case SUCCESS:
        led.lightOneSide(Side.RIGHT, 120);
      case ALIGNING:
        led.lightOneSide(Side.RIGHT, 60);
        break;
      case FAIL:
        led.lightOneSide(Side.RIGHT, 0);
        break;
      case IDLE:
        led.lightOneSide(Side.RIGHT, 30);
        break;
      case TIMEOUT:
        led.lightOneSide(Side.RIGHT, 140);
        break;
      default:
        led.lightOneSide(Side.RIGHT,20);
        break;
    }

    switch (RobotContainer.state.getDistanceState())
    {
      case SUCCESS:
        led.lightOneSide(Side.LEFT, 120);
      case ALIGNING:
        led.lightOneSide(Side.LEFT, 60);
        break;
      case FAIL:
        led.lightOneSide(Side.LEFT, 0);
        break;
      case IDLE:
        led.lightOneSide(Side.LEFT, 30);
        break;
      case TIMEOUT:
        led.lightOneSide(Side.LEFT, 140);
        break;
      default:
        led.lightOneSide(Side.LEFT,20);
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
