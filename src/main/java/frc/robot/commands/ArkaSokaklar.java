// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.LED.Side;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ArkaSokaklar extends SequentialCommandGroup {
  /** Creates a new ArkaSokaklar. */
  public ArkaSokaklar(LED led) {
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
      new RunCommand(() -> led.lightOneSide(Side.RIGHT, 130), led).withTimeout(0.03)
    );
  }
}
