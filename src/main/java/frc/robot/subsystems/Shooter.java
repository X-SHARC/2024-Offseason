// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

  CANSparkMax masterMotor = new CANSparkMax(31, MotorType.kBrushless);
  CANSparkMax slaveMotor = new CANSparkMax(40, MotorType.kBrushless);

  /** Creates a new Shooter. */
  public Shooter() {
    masterMotor.restoreFactoryDefaults();
    slaveMotor.restoreFactoryDefaults();

    masterMotor.setIdleMode(IdleMode.kCoast);
    slaveMotor.setIdleMode(IdleMode.kCoast);

    slaveMotor.setInverted(false);
    masterMotor.setInverted(false);

    slaveMotor.burnFlash();
    masterMotor.burnFlash();
  }

  public void shoot() {
    masterMotor.setVoltage(12);
    slaveMotor.setVoltage(12);
  }

  public void shootAmp() {
    masterMotor.setVoltage(-10);
    slaveMotor.setVoltage(-10);
  }

  public void stop() {
    masterMotor.setVoltage(0);
    slaveMotor.setVoltage(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
