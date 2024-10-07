// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Swerve extends SubsystemBase {

  public SwerveModule[] mSwerveMods;
  public Pigeon2 gyro;

  public SwerveDrivePoseEstimator swervePose;

  public PIDController swerveRotationalPID = new PIDController(0.12,0, 0);

  /** Creates a new Swerve. */
  public Swerve() {
    gyro = new Pigeon2(Constants.Swerve.pigeonID);
    gyro.getConfigurator().apply(new Pigeon2Configuration());
    gyro.setYaw(0);

    mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
    };

    swervePose = new SwerveDrivePoseEstimator(Constants.Swerve.swerveKinematics, getGyroYaw(), getModulePositions(), new Pose2d());

    AutoBuilder.configureHolonomic(
      this::getPose,
      this::setPose,
      this::getRRSpeeds,
      this::driveRR,
      new HolonomicPathFollowerConfig(
              new PIDConstants(2.2, 0.0, 0.0), // translation PID
              new PIDConstants(2.0, 0, 0), // rotation PID
              Constants.Swerve.kMaxSpeed,
              0.45461415,
              new ReplanningConfig()),
      () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
              return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
      },
      this
    );
  }    

  public Pose2d getPose() {
    return swervePose.getEstimatedPosition();
  }

  public void setPose(Pose2d pose) {
    swervePose.resetPosition(getGyroYaw(), getModulePositions(), pose);
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : mSwerveMods) {
        states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  public ChassisSpeeds getRRSpeeds() {
    return Constants.Swerve.kinematics.toChassisSpeeds(getModuleStates());
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
        mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

  public void driveRR(ChassisSpeeds speeds) {
    SwerveModuleState[] states = Constants.Swerve.kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.Swerve.kMaxSpeed);
    Constants.Swerve.kinematics.toChassisSpeeds(states);

    setModuleStates(states);
  }

  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (SwerveModule mod : mSwerveMods) {
        positions[mod.moduleNumber] = mod.getPosition();
    }
    return positions;
  }

  public Rotation2d getGyroYaw() {
    return Rotation2d.fromDegrees(gyro.getYaw().getValue());
  }

  public Rotation2d getHeading() {
    return getPose().getRotation();
  }

  public void zeroHeading() {
    swervePose.resetPosition(getGyroYaw(), getModulePositions(),
            new Pose2d(getPose().getTranslation(), new Rotation2d()));
  }

  public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
            fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    translation.getX(),
                    translation.getY(),
                    rotation,
                    getHeading())
                    : new ChassisSpeeds(
                            translation.getX(),
                            translation.getY(),
                            rotation));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
        mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }

  public double[] logSwerveState(){
    double[] log = new double[8];
    for (SwerveModule mod : mSwerveMods){
      log[mod.moduleNumber*2] = mod.getLogAngle();
      log[mod.moduleNumber*2+1] = mod.getLogVelocity();
    }
    return log;
  }

  public double[] logPosition(){
    Pose2d sp = swervePose.getEstimatedPosition();
    double[] pose = {sp.getX(), sp.getY(), sp.getRotation().getDegrees()};
    return pose;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    swervePose.update(getGyroYaw(), getModulePositions());
    // add limelight update
    SmartDashboard.putNumberArray("Pose Estimation", logPosition());
    SmartDashboard.putNumberArray("Swerve State", logSwerveState());
  }
}
