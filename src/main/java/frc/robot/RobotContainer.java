// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.TransferIntakeIdleCommand;
import frc.robot.commands.TransferIntakeIntakeCommand;
import frc.robot.commands.TransferIntakeOuttakeCommand;
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
    configureBindings();
  }

  private void configureBindings() {
    driverController.a().onTrue(shooterSubsystem.setFlywheelPowerCommand(0));
    driverController.b().onTrue(shooterSubsystem.setFlywheelVelocityCommand(0.2));
    driverController.x().whileTrue(shooterSubsystem.runFlywheelAtSpeedCommand(0.2));
    driverController.y().onTrue(shooterSubsystem.setFlywheelPowerCommand(0.3));

    //driverController.a().onTrue(transferSubsystem.ejectBalls());
    //driverController.b().onTrue(transferSubsystem.transferBalls());
    //driverController.x().onTrue(transferSubsystem.idle());
    //driverController.y().onTrue(transferSubsystem.manualOveride(0.4));

    driverController.povDown().onTrue(new TransferIntakeOuttakeCommand(transferSubsystem, intakeSubsystem));
    driverController.povUp().onTrue(new TransferIntakeIntakeCommand(transferSubsystem, intakeSubsystem));
    driverController.povRight().onTrue(new TransferIntakeIdleCommand(transferSubsystem, intakeSubsystem));
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
