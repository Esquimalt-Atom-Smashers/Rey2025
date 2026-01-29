// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveSlowModeCommand;
import frc.robot.commands.IdleSubsystemsCommand;
import frc.robot.commands.IntakeBallsCommand;
import frc.robot.commands.OuttakeBallsCommand;
import frc.robot.commands.RunShooterFeederCommand;
import frc.robot.commands.ToggleAimingHoodCommand;
import frc.robot.commands.ToggleShooterChargingCommand;
import frc.robot.subsystems.balltransfer.TransferSubsystem;
import frc.robot.subsystems.controlpanelrotator.CPRotatorSubsystem;
import frc.robot.subsystems.drivebase.DrivebaseSubsystem;
import frc.robot.subsystems.hang.HangingSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.ledlights.BlinkinSubsystem;
import frc.robot.subsystems.shooter.AimSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class RobotContainer{
  
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverController =
      new CommandXboxController(0);

  private IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private TransferSubsystem transferSubsystem = new TransferSubsystem();
  private ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private DrivebaseSubsystem drivebaseSubsystem = new DrivebaseSubsystem(() -> applyDeadzone(driverController.getLeftY()), 
                                                                         () -> applyDeadzone(driverController.getRightX()));
  private AimSubsystem aimSubsystem = new AimSubsystem();
  private HangingSubsystem hangingSubsystem = new HangingSubsystem();
  private BlinkinSubsystem ledSubsystem = new BlinkinSubsystem();
  private CPRotatorSubsystem cpRotatorSubsystem = new CPRotatorSubsystem();

  private final double controllerDeadzone = 0.2;

  public RobotContainer() {
    transferSubsystem.initializeSubsystem();
    intakeSubsystem.initializeSubsystem();
    shooterSubsystem.initializeSubsystem();
    drivebaseSubsystem.initializeSubsystem();
    aimSubsystem.initializeSubsystem();

    configureBindings();
  }

  private void configureBindings() {
    // -- Intake and Ball Transfer --
    driverController.leftBumper().whileTrue(new OuttakeBallsCommand(intakeSubsystem, transferSubsystem));
    driverController.leftTrigger().whileTrue(new IntakeBallsCommand(intakeSubsystem, transferSubsystem));

    // -- Shooting Controls --
    // Run shooter feeder while holding
    driverController.rightTrigger().whileTrue(new RunShooterFeederCommand(shooterSubsystem));

    // Toggle between charging and idle
    driverController.x().onTrue(new ToggleShooterChargingCommand(shooterSubsystem));
    // driverController.x().onTrue(shooterSubsystem.setFlywheelPowerCommand(1));
    // driverController.x().onFalse(shooterSubsystem.setFlywheelPowerCommand(0));

    // Adjust velocity
    driverController.povUp()   .onTrue(shooterSubsystem.setTargetFlywheelVelocity(shooterSubsystem.FAST_FLYWHEEL_VELOCITY));
    driverController.povRight().onTrue(shooterSubsystem.setTargetFlywheelVelocity(shooterSubsystem.DEFAULT_FLYWHEEL_VELOCITY));
    driverController.povDown() .onTrue(shooterSubsystem.setTargetFlywheelVelocity(shooterSubsystem.SLOW_FLYWHEEL_VELOCITY));

    // Aiming panel
    driverController.y().onTrue(new ToggleAimingHoodCommand(aimSubsystem));

    driverController.b().onTrue(new InstantCommand(() -> {
      aimSubsystem.setTargetPosition(AimSubsystem.hoodDownPosition);
    }));

    driverController.a().onTrue(new InstantCommand(() -> {
      aimSubsystem.setTargetPosition(AimSubsystem.hoodUpPosition);
    }));

    // -- Drive Controls --
    driverController.rightBumper().whileTrue(new DriveSlowModeCommand(drivebaseSubsystem));

    // -- Idle all systems --
    driverController.start().onTrue(new IdleSubsystemsCommand(transferSubsystem, intakeSubsystem, shooterSubsystem, aimSubsystem));
  }

  public double applyDeadzone(double value){
    if (Math.abs(value) < controllerDeadzone) {
      return 0.0;
    } 
    else {
      double sign = Math.signum(value);
      double scaledValue = (Math.abs(value) - controllerDeadzone) / (1.0 - controllerDeadzone);

      return sign * scaledValue;
    }
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
    aimSubsystem.initializeSubsystem();
  }

  public void disabledInit() {
    shooterSubsystem.shutdownSubsystem();
    // TODO: ADD MORE SYSTEMS
  }
}
