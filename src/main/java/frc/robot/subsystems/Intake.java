// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  CANSparkMax intakeMotor = new CANSparkMax(45, MotorType.kBrushless);

  /** Creates a new Intake. */
  public Intake() {
    intakeMotor.restoreFactoryDefaults();
    intakeMotor.setInverted(true);
    intakeMotor.setIdleMode(IdleMode.kBrake);
    intakeMotor.burnFlash();
  }

  public void intakeIn() {
    intakeMotor.setVoltage(10);
  }

  public void intakeOut() {
    intakeMotor.setVoltage(-10);
  }

  public void stop() {
    intakeMotor.setVoltage(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
