// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;

public class Feeder extends SubsystemBase {

  CANSparkMax feederMotor = new CANSparkMax(44, MotorType.kBrushless);
  DigitalInput beamSensor = new DigitalInput(4);

  /** Creates a new Feeder. */
  public Feeder() {
    feederMotor.restoreFactoryDefaults();
    feederMotor.setInverted(false);
    feederMotor.setIdleMode(IdleMode.kBrake);
    feederMotor.burnFlash();
  }

  public void feedIn() {
    feederMotor.setVoltage(12);
  }

  public void feedOut() {
    feederMotor.setVoltage(-12);
  }

  public void stop() {
    feederMotor.setVoltage(0);
  }

  public boolean getBeamSensor() {
    return beamSensor.get();
  }

  @Override
  public void periodic() {
    if (getBeamSensor()) {
      RobotState.setObjectPresent();
    }else {
      RobotState.setObjectAbsent();
    }
    // This method will be called once per scheduler run
  }
}
