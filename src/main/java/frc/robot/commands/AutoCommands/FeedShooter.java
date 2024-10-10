// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feeder;

public class FeedShooter extends Command {
  Feeder feeder;
  double counter = 0;

  /** Creates a new FeedShooter. */
  public FeedShooter(Feeder feeder) {
    this.feeder = feeder;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    counter = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    feeder.feedIn();
    counter += 0.02;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    feeder.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ((!feeder.getBeamSensor() || counter >= 0.85) && counter >= 0.3);
  }
}
