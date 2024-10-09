// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  TalonFX armMasterMotor = new TalonFX(19);
  TalonFX armSlaveMotor = new TalonFX(32);

  private double ANGULAR_RANGE_ARM = 110;
  // TODO: Forward veya Reverse olarak değiştirilecek, Ters yöne de limit switch
  // takılıp Reset eklenecek, Amper Limit eklenecek

  TalonFXConfiguration masterConfig = new TalonFXConfiguration();
  TalonFXConfiguration slaveConfig = new TalonFXConfiguration();

  public DigitalInput limitSwitch = new DigitalInput(3);

  SlewRateLimiter armLimiter = new SlewRateLimiter(20);

  /*
   * amp için
   * arm: 80
   * 69.84 / 72
   * 
   * shooter backward 0.6
   * feeder 1
   */

  public CANcoder armEncoder = new CANcoder(35);

  private double kP = 0.027;
  private double kI = 0.0;
  private double kD = 0.0;

  private double angle = 0;

  private PIDController armPID = new PIDController(kP, kI, kD);

  double GearRatio1 = 1 / 148.138666667; // bakarsın
  double offset = 1;

  // alt limit 12.1 - 8.3 p: 0.085

  /** Creates a new Arm. */
  public Arm() {
    ANGULAR_RANGE_ARM = ANGULAR_RANGE_ARM / 360;
    SoftwareLimitSwitchConfigs limitConfig = new SoftwareLimitSwitchConfigs()
        .withForwardSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(ANGULAR_RANGE_ARM / GearRatio1);

    armMasterMotor.setInverted(true);

    armMasterMotor.setNeutralMode(NeutralModeValue.Brake);
    armSlaveMotor.setNeutralMode(NeutralModeValue.Brake);
    armMasterMotor.getConfigurator().apply(limitConfig);
    armSlaveMotor.getConfigurator().apply(limitConfig);
    armMasterMotor.setControl(new Follower(armSlaveMotor.getDeviceID(), true));

    armPID.setTolerance(0.5);
  }

  public void armUp() {
    armMasterMotor.set(armLimiter.calculate(0.5));
    armSlaveMotor.set(armLimiter.calculate(0.5));
  }

  public void armDown() {
    if (limitSwitch.get() == true) {
      armMasterMotor.set(armLimiter.calculate(-0.3));
      armSlaveMotor.set(armLimiter.calculate(-0.3));
    }

  }

  public void stop() {
    armMasterMotor.set(0.0);
    armSlaveMotor.set(0.0);
  }

  public double getDegrees() {
    angle = armMasterMotor.getPosition().getValue();
    angle = angle * GearRatio1;

    return angle * 360;
  }

  public void setAngle(double Setpoint) {
    double currAngle = getDegrees();
    double PIDOutput = MathUtil.clamp(armPID.calculate(currAngle, Setpoint), -0.8, 0.8);
    armMasterMotor.set(armLimiter.calculate((PIDOutput * 12.) / RobotController.getBatteryVoltage()));
    armSlaveMotor.set(armLimiter.calculate((PIDOutput * 12.) / RobotController.getBatteryVoltage()));
  }

  public void resetEncoder() {
    armMasterMotor.getConfigurator().setPosition(0);
    armSlaveMotor.getConfigurator().setPosition(0);
  }

  @Override
  public void periodic() {
    if (limitSwitch.get() == false) {
      armMasterMotor.set(0.0);
      resetEncoder();
    }

    SmartDashboard.putNumber("Arm Angle", getDegrees());
    SmartDashboard.putBoolean("Arm Limit Switch", limitSwitch.get());

    // This method will be called once per scheduler run
  }
}
