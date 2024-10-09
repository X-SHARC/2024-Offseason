package frc.robot;

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

    public enum ArmState {
        ZERO,
        AIMING,
        AIMED
    }

    public enum Object{
        PRESENT, 
        ABSENT
    }

    public enum ArmBroken{
        BROKEN,
        FIXED
    }

    public enum LockIn{
        LOCKED,
        FREE
    }


    public static SwerveState swerveState = SwerveState.NO_ALIGN;
    public static TargetState targetState = TargetState.ABSENT;
    public static Object objectState = Object.ABSENT;
    public static ArmBroken armBroken = ArmBroken.FIXED;
    public static LockIn lockIn = LockIn.FREE;



    public static RobotState getInstance(){
        if(robotState == null){
            robotState = new RobotState();
        }
        return robotState;
    }
}
