package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


public class TeleopSwerve extends Command {    
    private Swerve s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;

    public PIDController SwerveRotatePID = new PIDController(0.018,0,0);

    @SuppressWarnings("unused")
    private CommandXboxController driver;

    public TeleopSwerve(CommandXboxController driver, Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
        this.driver = driver;
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);

        if (RobotState.lockIn == RobotState.LockIn.LOCKED){
            
            Pose2d pose = s_Swerve.swervePose.getEstimatedPosition();
            double x = pose.getX();
            double y = pose.getY();
            double goal;

            if (y < 5.5){
                double thetaRad = Math.atan(-1*(y-5.5) /x);
                double thetaDegrees = Math.toDegrees(thetaRad);
                goal = thetaDegrees * -1;
                SmartDashboard.putNumber("goal", thetaDegrees);
            }
            else{
                double thetaRad = Math.atan((y-5.5) /x);
                double thetaDegrees = Math.toDegrees(thetaRad);
                
                goal = thetaDegrees;
                SmartDashboard.putNumber("goal", thetaDegrees);
            }

            s_Swerve.drive(
                new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
                SwerveRotatePID.calculate(s_Swerve.getGyroDouble(), goal) * Constants.Swerve.maxAngularVelocity, 
                !robotCentricSup.getAsBoolean(), 
            true
            ); 
            
        } else{
            /* Drive */

            s_Swerve.drive(
                new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
                rotationVal * Constants.Swerve.maxAngularVelocity, 
                !robotCentricSup.getAsBoolean(), 
            true
            );  
        }
        
        
        
    }
}