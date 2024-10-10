package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final double stickDeadband = 0.1;

    public static final class Swerve {
        public static final int pigeonID = 23;

        public static final COTSTalonFXSwerveConstants chosenModule = // TODO: This must be tuned to specific robot
                COTSTalonFXSwerveConstants.SDS.MK4i.Falcon500(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L2);

        /* Drivetrain Constants */
        public static final double trackWidth = 0.45461415; // TODO: This must be tuned to specific robot
        public static final double wheelBase = 0.45461415; // TODO: This must be tuned to specific robot
        public static final double wheelCircumference = 2 * Math.PI * Units.inchesToMeters(2.03);

        /*
         * Swerve Kinematics
         * No need to ever change this unless you are not doing a traditional
         * rectangular/square 4 module swerve
         */
        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
        public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

        /* Swerve Current Limiting */
        public static final int angleCurrentLimit = 25;
        public static final int angleCurrentThreshold = 25;
        public static final double angleCurrentThresholdTime = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveCurrentLimit = 35;
        public static final int driveCurrentThreshold = 40;
        public static final double driveCurrentThresholdTime = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /*
         * These values are used by the drive falcon to ramp in open loop and closed
         * loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc
         */
        public static final double openLoopRamp = 0.1;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;

        /* Drive Motor PID Values */
        public static final double driveKP = 2.432; // TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double driveKS = 2.3289 / 10; // TODO: This must be tuned to specific robot
        public static final double driveKV = 2.3289 / 10;
        public static final double driveKA = 0.31447 / 10;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = Units.feetToMeters(15.2); // TODO: This must be tuned to specific robot
        /** Radians per Second */
        public static final double maxAngularVelocity = 2 * Math.PI; // TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { // TODO: This must be tuned to specific robot
            public static final int driveMotorID = 18;
            public static final int angleMotorID = 17;
            public static final int canCoderID = 2;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(199.252);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { // TODO: This must be tuned to specific robot
            public static final int driveMotorID = 11;
            public static final int angleMotorID = 14;
            public static final int canCoderID = 1;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(45.0 + 180.0 + 2.988);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 { // TODO: This must be tuned to specific robot
            public static final int driveMotorID = 10;
            public static final int angleMotorID = 16;
            public static final int canCoderID = 4;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(121.0 + 180.0);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { // TODO: This must be tuned to specific robot
            public static final int driveMotorID = 13;
            public static final int angleMotorID = 12;
            public static final int canCoderID = 3;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(317.82);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        public static final double kMaxSpeed = Units.feetToMeters(15.2); // 16.2 feet per second
        public static final double kMaxAngularSpeed = 2 * Math.PI; // 1/2 rotation per second
        public static final double kModuleMaxAngularAcceleration = 2 * Math.PI; // radians per second squared

        public static final double kLength = 0.45461415;
        public static final double kWidth = 0.45461415;
        // 0.44229515;
        // 0.45461415;
        // public static final double kLength = 0.75;
        // public static final double kWidth = 0.75;

        public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
                new Translation2d(kLength / 2., kWidth / 2.),
                new Translation2d(kLength / 2., -kWidth / 2.),
                new Translation2d(-kLength / 2., kWidth / 2.),
                new Translation2d(-kLength / 2., -kWidth / 2.));

    }

    public static class ArmData {
        public static final InterpolatingDoubleTreeMap interpolatingMap = new InterpolatingDoubleTreeMap();
        static {
            //length, arm angle
            interpolatingMap.put(135.0, 0.0 ); 
            interpolatingMap.put(174.0, 6.0 ); 
            interpolatingMap.put(193.0, 8.0 ); 
            interpolatingMap.put(206.0, 10.0);
            interpolatingMap.put(236.0, 12.0);
            interpolatingMap.put(256.0, 14.0);
            interpolatingMap.put(284.0, 16.0);
            interpolatingMap.put(340.0, 17.0);
            interpolatingMap.put(375.0, 18.0);
            interpolatingMap.put(412.0, 19.0);
            interpolatingMap.put(405.0, 20.0);
            interpolatingMap.put(502.0, 20.7);
        }
    }

    public static final boolean kGyroReversed = false;

    public static final double LIMELIGHT_HEIGHT = 25.7137;
    public static final double LIMELIGHT_MOUNTING_ANGLE = 34; // 37,4
    public static final double MOUNTING_POSITION = 0;

    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;
    }

    public static class Field {
        public static final int[] speakerAprilTags = { 4, 7 };
        public static final int[] speakerSideAprilTags = { 3, 8 };
        public static final int[] ampAprilTags = { 5, 6 };
        public static final double ampAprilHeight = 126.5; // 122
        public static final double speakerAprilHeight = 144;

    }
}
