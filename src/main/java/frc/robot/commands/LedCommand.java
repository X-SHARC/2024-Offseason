// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.subsystems.LED;

public class LedCommand extends Command {

  LED led;

  /** Creates a new LedCommand. */
  public LedCommand(LED led) {
    this.led = led;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /* LED COLOR CODES:
        * aligned to target -> orange (solid)
        * aligning to target -> purple (blink)
        * object present -> yellow (blink)
        * default -> green (sliding)
    */
    if (RobotState.getSwerveState() == RobotState.SwerveState.ALIGNED){
      led.setColor(255, 87, 51); // orange
    }
    else if (RobotState.getSwerveState() == RobotState.SwerveState.ALIGNING){
      led.blink(160, 32, 240); // purple
    }
    else if (RobotState.getObjectState() == RobotState.Object.PRESENT){
      led.blink(252, 127, 3); // yellow
    }
    else{
      led.sliding(Color.kGreen); // green
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
