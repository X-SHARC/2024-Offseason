package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.RobotState;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

import edu.wpi.first.math.kinematics.SwerveModulePosition;

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
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;
    public int updateCount = 0;
    private double[] llPose = { 0, 0, 0 };

    public SwerveDrivePoseEstimator swervePose;

    public PIDController swerveRotationalPID = new PIDController(0.12, 0, 0);

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

        swervePose = new SwerveDrivePoseEstimator(Constants.Swerve.swerveKinematics, getGyroYaw(), getModulePositions(),
                new Pose2d());

        AutoBuilder.configureHolonomic(
                this::getPose,
                this::setPose,
                this::getRRSpeeds,
                this::driveRR,
                new HolonomicPathFollowerConfig(
                        // 1.5 - 2.0
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
                this);
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

    public void driveRR(ChassisSpeeds speeds) {
        SwerveModuleState[] states = Constants.Swerve.kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.Swerve.kMaxSpeed);
        Constants.Swerve.kinematics.toChassisSpeeds(states);

        setModuleStates(states);
    }

    public ChassisSpeeds getRRSpeeds() {
        return Constants.Swerve.kinematics.toChassisSpeeds(getModuleStates());
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : mSwerveMods) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : mSwerveMods) {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public Pose2d getPose() {
        return swervePose.getEstimatedPosition();
    }

    public double getGyroDouble() {
        return gyro.getYaw().getValueAsDouble();
    }

    public void setPose(Pose2d pose) {
        swervePose.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    public void setHeading(Rotation2d heading) {
        swervePose.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading() {
        swervePose.resetPosition(getGyroYaw(), getModulePositions(),
                new Pose2d(getPose().getTranslation(), new Rotation2d()));

    }

    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(gyro.getYaw().getValue());
    }

    public void resetModulesToAbsolute() {
        for (SwerveModule mod : mSwerveMods) {
            mod.resetToAbsolute();
        }
    }

    public void updateFromVisionMT1() {

        boolean doRejectUpdate = false;

        LimelightHelpers.PoseEstimate mt1 = 
            RobotState.isBlueAlliance() ? 
            LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-sharc") : 
            LimelightHelpers.getBotPoseEstimate_wpiRed("limelight-sharc");
        
        if (mt1.tagCount == 1 && mt1.rawFiducials.length == 1) {
            if (mt1.rawFiducials[0].ambiguity > .7) {
                doRejectUpdate = true;
            }
            if (mt1.rawFiducials[0].distToCamera > 3) {
                doRejectUpdate = true;
            }
        }
        if (mt1.tagCount == 0) {
            doRejectUpdate = true;
        }

        if (!doRejectUpdate) {
            // swervePose.setVisionMeasurementStdDevs(VecBuilder.fill(.5,.5,9999999));
            llPose[0] = mt1.pose.getX();
            llPose[1] = mt1.pose.getY();
            llPose[2] = mt1.pose.getRotation().getDegrees() < 0 ? mt1.pose.getRotation().getDegrees() + 180
                    : mt1.pose.getRotation().getDegrees() - 180;

            swervePose.addVisionMeasurement(
                    new Pose2d(new Translation2d(mt1.pose.getX(), mt1.pose.getY()), getGyroYaw()),
                    mt1.timestampSeconds);
        }

    }

    private double[] logState() {
        double[] state = new double[8];
        for (int i = 0; i < 4; i++) {
            state[i * 2] = mSwerveMods[i].getLogAngle();
            state[i * 2 + 1] = mSwerveMods[i].getLogVelocity();
        }
        return state;
    }

    @Override
    public void periodic() {
        swervePose.update(getGyroYaw(), getModulePositions());

        updateFromVisionMT1();

        SmartDashboard.putNumberArray("SwerveModuleStates", logState());

        SmartDashboard.putNumberArray("LL Robot Pose", llPose);

        Pose2d sp = swervePose.getEstimatedPosition();
        double[] pose = { sp.getX(), sp.getY(), sp.getRotation().getDegrees() };
        SmartDashboard.putNumberArray("Pose Estimator Robot Pose", pose);

    }
}