package frc.robot;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class RobotState {
    private static RobotState robotState;

    public enum SwerveState {
        NO_ALIGN,
        ALIGNING,
        ALIGNED
    }

    public enum TargetState {
        PRESENT,
        ABSENT
    }

    public enum Object{
        PRESENT, 
        ABSENT
    }

    public enum LockIn{
        LOCKED,
        FREE,
        DISABLED
    }



    public static SwerveState swerveState = SwerveState.NO_ALIGN;
    public static TargetState targetState = TargetState.ABSENT;
    public static Object objectState = Object.ABSENT;
    public static LockIn lockIn = LockIn.DISABLED;
    public static Optional<Alliance> alliance = DriverStation.getAlliance();



    public static RobotState getInstance(){
        if(robotState == null){
            robotState = new RobotState();
        }
        return robotState;
    }
    
    public static void setAligned(){
        swerveState = SwerveState.ALIGNED;
    }

    public static void setNoAlign(){
        swerveState = SwerveState.NO_ALIGN;
    }

    public static void setAligning(){
        swerveState = SwerveState.ALIGNING;
    }


    public static void setTargetPresent(){
        targetState = TargetState.PRESENT;
    }

    public static void setTargetAbsent(){
        targetState = TargetState.ABSENT;
    }


    public static void setObjectPresent(){
        objectState = Object.PRESENT;
    }

    public static void setObjectAbsent(){
        objectState = Object.ABSENT;
    }


    public static void setLockIn(){
        lockIn = LockIn.LOCKED;
    }

    public static void setFree(){
        lockIn = LockIn.FREE;
    }

    
    public static SwerveState getSwerveState(){
        return swerveState;
    }

    public static TargetState getTargetState(){
        return targetState;
    }

    public static Object getObjectState(){
        return objectState;
    }

    public static LockIn getLockIn(){
        return lockIn;
    }

    public static boolean canLockIn(){
        return lockIn != LockIn.DISABLED;
    }

    public static void toggleLockInDisabled(){
        if(lockIn == LockIn.DISABLED){
            lockIn = LockIn.FREE;
        } else {
            lockIn = LockIn.DISABLED;
        }
    }
}
