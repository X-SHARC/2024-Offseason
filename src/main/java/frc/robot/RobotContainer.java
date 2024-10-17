// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AmpSequence;
import frc.robot.commands.ArkaSokaklar;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.AutoAim;
import frc.robot.commands.EjectNote;
import frc.robot.commands.GetNote;
import frc.robot.commands.LedCommand;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.AutoCommands.FeedShooter;
import frc.robot.commands.AutoCommands.GrabAndFeed;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  
  private final Swerve m_swerve = new Swerve();
  private final Intake intake = new Intake();
  private final Feeder feeder = new Feeder();
  private final Shooter shooter = new Shooter();
  private final Arm arm = new Arm();
  private final LED led = new LED(1, 60);

  // Commands
  GetNote getNoteCommand = new GetNote(intake, feeder);
  EjectNote ejectNoteCommand = new EjectNote(intake, feeder);

  AmpSequence ampSequenceCommand = new AmpSequence(arm, shooter, feeder);
  ArmCommand armClosedCommand = new ArmCommand(arm, 5);
  RunCommand armUpCommand = new RunCommand(() -> arm.armUp(), arm);
  RunCommand armDownCommand = new RunCommand(() -> arm.armDown(), arm);
  InstantCommand armStopCommand = new InstantCommand(() -> arm.stop(), arm);

  RunCommand shooterSpeedUpCommand = new RunCommand(() -> shooter.shoot(), shooter);
  InstantCommand shooterStopCommand = new InstantCommand(() -> shooter.stop(), shooter);

  RunCommand feederInCommand = new RunCommand(() -> feeder.feedIn(), feeder);
  InstantCommand feederStopCommand = new InstantCommand(() -> feeder.stop(), feeder);

  InstantCommand gyroResetCommand = new InstantCommand(() -> m_swerve.zeroHeading(), m_swerve);

  AutoAim autoAimCommand = new AutoAim(arm, m_swerve);

  ArkaSokaklar arkaSokaklarCommand = new ArkaSokaklar(led);
  
  // Auto Commands
  GrabAndFeed grabAndFeed = new GrabAndFeed(intake, feeder);
  FeedShooter feedShooter = new FeedShooter(feeder);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private final CommandPS5Controller m_operatorController = 
      new CommandPS5Controller(OperatorConstants.kOperatorControllerPort);


  private final SendableChooser<Command> autoChooser;
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);

    NamedCommands.registerCommand("intake", grabAndFeed);
    NamedCommands.registerCommand("spinShooter", shooterSpeedUpCommand);
    NamedCommands.registerCommand("feedShooter", feedShooter);

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    m_swerve.setDefaultCommand(
      new TeleopSwerve(
        null,
        m_swerve, 
        () -> -m_driverController.getLeftY(), 
        () -> -m_driverController.getLeftX(), 
        () -> -m_driverController.getRightX(), 
        () -> false));

    led.setDefaultCommand(new LedCommand(led));

    m_driverController.a().whileTrue(getNoteCommand);
    m_driverController.b().onTrue(gyroResetCommand);
    m_driverController.button(8).onTrue(gyroResetCommand); // option button
    m_driverController.rightBumper()
      .whileTrue(feederInCommand)
      .onFalse(feederStopCommand);

    m_driverController.leftBumper()
      .whileTrue(autoAimCommand)
      .whileTrue(shooterSpeedUpCommand)
      .onFalse(armStopCommand)
      .onFalse(shooterStopCommand)
      .onTrue(new InstantCommand(() -> {if(RobotState.canLockIn()){RobotState.setLockIn();}}))
      .onFalse(new InstantCommand(() -> RobotState.setFree()));

    // Operator Controls
    m_operatorController.R1()
      .whileTrue(ampSequenceCommand)
      .onFalse(shooterStopCommand)
      .onFalse(feederStopCommand)
      .onFalse(armClosedCommand);
    m_operatorController.circle().whileTrue(getNoteCommand);
    m_operatorController.square().whileTrue(ejectNoteCommand);
    m_operatorController.triangle().whileTrue(armUpCommand).onFalse(armStopCommand);
    m_operatorController.cross().whileTrue(armDownCommand).onFalse(armStopCommand);
    m_operatorController.R3().whileTrue(arkaSokaklarCommand);
    m_operatorController.L1().onTrue(new InstantCommand(() -> RobotState.toggleLockInDisabled()));


    // // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition)
    //     .onTrue(new ExampleCommand(m_exampleSubsystem));

    // // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // // cancelling on release.
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return Autos.exampleAuto(m_exampleSubsystem);
    return autoChooser.getSelected();
  }
}
