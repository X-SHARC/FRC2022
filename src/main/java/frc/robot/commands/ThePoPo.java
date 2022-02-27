// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.lib.drivers.WS2812Driver;
import frc.robot.lib.drivers.WS2812Driver.Side;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ThePoPo extends SequentialCommandGroup {
  private WS2812Driver led;

  /** Creates a new PoPo. */
  public ThePoPo(WS2812Driver led) {
    this.led = led;
    System.out.println("the po-lice is coming");
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new RunCommand(() -> led.lightOneSide(Side.LEFT, 0), led).withTimeout(0.03),
        new RunCommand(() -> led.turnOff(), led).withTimeout(0.03),
        new RunCommand(() -> led.lightOneSide(Side.LEFT, 0), led).withTimeout(0.03),
        new RunCommand(() -> led.turnOff(), led).withTimeout(0.03),
        new RunCommand(() -> led.lightOneSide(Side.LEFT, 0), led).withTimeout(0.03),
        new RunCommand(() -> led.lightOneSide(Side.RIGHT, 130), led).withTimeout(0.03),
        new RunCommand(() -> led.turnOff(), led).withTimeout(0.03),
        new RunCommand(() -> led.lightOneSide(Side.RIGHT, 130), led).withTimeout(0.03),
        new RunCommand(() -> led.turnOff(), led).withTimeout(0.03),
        new RunCommand(() -> led.lightOneSide(Side.RIGHT, 130), led).withTimeout(0.03));
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    led.turnOff();
  }
}
