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
    switch (RobotContainer.state.getAlignmentState()) {
      case SUCCESS:
        led.setAllLeds(Side.RIGHT, 0, 255, 0);
        break;
      case ALIGNING:
        led.setAllLeds(Side.RIGHT, 0, 255, 100);
        break;
      case FAIL:
        led.setAllLeds(Side.RIGHT, 255, 0, 0);
        break;
      case IDLE:
        led.setAllLeds(Side.RIGHT, 255, 255, 100);
        break;
      case TIMEOUT:
        led.setAllLeds(Side.RIGHT, 0, 40, 0);
        break;
      default:
        led.setAllLeds(Side.RIGHT, 0, 0, 0);
        break;
    }

    switch (RobotContainer.state.getDistanceState()) {
      case SUCCESS:
        led.setAllLeds(Side.LEFT, 0, 255, 0);
        break;
      case ALIGNING:
        led.setAllLeds(Side.LEFT, 0, 255, 100);
        break;
      case FAIL:
        led.setAllLeds(Side.LEFT, 255, 0, 0);
        break;
      case IDLE:
        led.setAllLeds(Side.LEFT, 255, 255, 100);
        break;
      case TIMEOUT:
        led.setAllLeds(Side.LEFT, 0, 40, 0);
        break;
      default:
        led.setAllLeds(Side.LEFT, 0, 0, 0);
        break;
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
