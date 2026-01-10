// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.IdleSubsystemsCommand;
import frc.robot.commands.IntakeBallsCommand;
import frc.robot.commands.OuttakeBallsCommand;
import frc.robot.commands.RunShooterFeederCommand;
import frc.robot.subsystems.balltransfer.TransferSubsystem;
import frc.robot.subsystems.controlpanelrotator.CPRotatorSubsystem;
import frc.robot.subsystems.drivebase.DrivebaseSubsystem;
import frc.robot.subsystems.hang.HangingSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.ledlights.BlinkinSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class RobotContainer{
  
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverController =
      new CommandXboxController(0);

  private IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private TransferSubsystem transferSubsystem = new TransferSubsystem();
  private ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private DrivebaseSubsystem drivebaseSubsystem = new DrivebaseSubsystem();
  private HangingSubsystem hangingSubsystem = new HangingSubsystem();
  private BlinkinSubsystem ledSubsystem = new BlinkinSubsystem();
  private CPRotatorSubsystem cpRotatorSubsystem = new CPRotatorSubsystem();

  public RobotContainer() {
    transferSubsystem.initializeSubsystem();
    intakeSubsystem.initializeSubsystem();
    shooterSubsystem.initializeSubsystem();
    configureBindings();
  }

  private void configureBindings() {
    // -- Intake and Ball Transfer --
    driverController.a().whileTrue(new OuttakeBallsCommand(intakeSubsystem, transferSubsystem));
    driverController.b().whileTrue(new IntakeBallsCommand(intakeSubsystem, transferSubsystem));

    // -- Shooting Controls --

    driverController.b().onTrue(new InstantCommand(() -> {
      intakeSubsystem.intake();
    }));
    // Run shooter feeder while holding
    driverController.x().whileTrue(new RunShooterFeederCommand(shooterSubsystem));

    // Toggle shooter flywheel on/off
    boolean shooterRunning = shooterSubsystem.getState() == ShooterSubsystem.ShooterSubsystemStates.charging || shooterSubsystem.getState() == ShooterSubsystem.ShooterSubsystemStates.feeding;
    if (shooterRunning) {
      driverController.y().onTrue(new InstantCommand(() -> { shooterSubsystem.idle(); } ));
    } else {
      driverController.y().onTrue(new InstantCommand(() -> { shooterSubsystem.charging(); } ));
    }
    driverController.povUp().onTrue(shooterSubsystem.setCurrentFlywheelVelocity(300));

    // -- Idle all systems --
    driverController.start().onTrue(new IdleSubsystemsCommand(transferSubsystem, intakeSubsystem, shooterSubsystem));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
  
  public void initializeSubsystems() {
    intakeSubsystem.initializeSubsystem();
    transferSubsystem.initializeSubsystem();
    shooterSubsystem.initializeSubsystem();
    drivebaseSubsystem.initializeSubsystem();
    hangingSubsystem.initializeSubsystem();
    ledSubsystem.initializeSubsystem();
    cpRotatorSubsystem.initializeSubsystem();    
  }
}
