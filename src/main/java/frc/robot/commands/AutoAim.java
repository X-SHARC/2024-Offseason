// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Swerve;

public class AutoAim extends Command {

  Swerve swerve;
  Arm arm;
  double distance;
  double setpoint;
  

  /** Creates a new AutoAim. */
  public AutoAim(Arm arm, Swerve swerve) {
    this.arm = arm;
    this.swerve = swerve;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d p = swerve.swervePose.getEstimatedPosition();
    distance = Math.sqrt(Math.pow(p.getY()-5.5,2) + Math.pow(p.getX(), 2)) * 100;
    setpoint = Constants.ArmData.interpolatingMap.get(distance);
    SmartDashboard.putNumber("arm set", setpoint);
    arm.setAngle(setpoint);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
