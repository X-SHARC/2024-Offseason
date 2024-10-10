// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;

public class GrabAndFeed extends Command {
  Intake intake;
  Feeder feeder;
  double counter = 0;

  /** Creates a new GrabAndFeed. */
  public GrabAndFeed(Intake intake, Feeder feeder) {
    this.feeder = feeder;
    this.intake = intake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake, feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    counter = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.intakeIn();
    feeder.feedIn();
    counter += 0.02;

    if (counter >= 4.0) {
      intake.intakeOut();
      feeder.feedOut();
    }
    SmartDashboard.putNumber("timer", counter);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stop();
    feeder.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (feeder.getBeamSensor() || counter > 3.20);
  }
}
