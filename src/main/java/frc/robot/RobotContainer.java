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
import frc.robot.commands.RunShooterFeederCommand;
import frc.robot.commands.ShuffleBallsCommand;
import frc.robot.commands.ToggleAimingHoodCommand;
import frc.robot.commands.ToggleShooterChargingCommand;
import frc.robot.subsystems.balltransfer.TransferSubsystem;
import frc.robot.subsystems.balltransfer.TransferSubsystem.TransferSubsystemStates;
import frc.robot.subsystems.controlpanelrotator.CPRotatorSubsystem;
import frc.robot.subsystems.controlpanelrotator.CPRotatorSubsystem.CPRotatorSubsystemStates;
import frc.robot.subsystems.drivebase.DrivebaseSubsystem;
import frc.robot.subsystems.hang.HangingSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem.IntakeSubsystemStates;
import frc.robot.subsystems.ledlights.BlinkinSubsystem;
import frc.robot.subsystems.shooter.AimSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooter.AimSubsystem.AimingSubsystemStates;

public class RobotContainer {
  
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverController1 =
      new CommandXboxController(0);
  private final CommandXboxController driverController2 =
      new CommandXboxController(1);

  private IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private TransferSubsystem transferSubsystem = new TransferSubsystem();
  private ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private DrivebaseSubsystem drivebaseSubsystem = new DrivebaseSubsystem(() -> applyDeadzone(driverController1.getLeftY()), 
                                                                         () -> applyDeadzone(driverController1.getLeftX()));
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
    aimSubsystem.setTargetState(AimingSubsystemStates.AIMED);
    cpRotatorSubsystem.initializeSubsystem();

    configureBindings();
  }

  private void configureBindings() {
    //region Driver 1
    driverController1.rightTrigger().whileTrue(new DriveSlowModeCommand(drivebaseSubsystem));

    //region Driver 2
    //region Intake/Outtake
    driverController2.leftBumper().whileTrue(new IntakeBallsCommand(intakeSubsystem, transferSubsystem, IntakeSubsystemStates.OUTTAKING, TransferSubsystemStates.EJECT));
    driverController2.leftTrigger().whileTrue(new IntakeBallsCommand(intakeSubsystem, transferSubsystem, IntakeSubsystemStates.INTAKING, TransferSubsystemStates.TRANSFER));
    driverController2.rightBumper().whileTrue(new ShuffleBallsCommand(intakeSubsystem, transferSubsystem));
    //endregion

    //region Shooting
    driverController2.rightTrigger().whileTrue(new RunShooterFeederCommand(shooterSubsystem));

    // Shooting Settings 
    driverController2.y() .onTrue(shooterSubsystem.setTargetFlywheelVelocity(shooterSubsystem.FAST_FLYWHEEL_VELOCITY));
    driverController2.b() .onTrue(shooterSubsystem.setTargetFlywheelVelocity(shooterSubsystem.DEFAULT_FLYWHEEL_VELOCITY));
    driverController2.x() .onTrue(shooterSubsystem.setTargetFlywheelVelocity(shooterSubsystem.SLOW_FLYWHEEL_VELOCITY));
    driverController2.a() .onTrue(shooterSubsystem.setTargetFlywheelVelocity(shooterSubsystem.SLOWER_FLYWHEEL_VECLOTY));

    driverController2.povUp().onTrue(new InstantCommand(() -> {
      aimSubsystem.setTargetPosition(AimSubsystem.hoodUpPosition);
    }));

    driverController2.povRight().onTrue(new InstantCommand(() -> {
      aimSubsystem.setTargetPosition(AimSubsystem.hoodHalfwayPosition);
    }));

    driverController2.povDown().onTrue(new InstantCommand(() -> {
      aimSubsystem.setTargetPosition(AimSubsystem.hoodDownPosition);
    }));

    driverController2.povLeft().onTrue(new InstantCommand(() -> {
       cpRotatorSubsystem.setVoltage(0.5);
    }));
    //endregion

    //region IDLE ALL SYSTEMS
    driverController2.start().onTrue(new IdleSubsystemsCommand(transferSubsystem, intakeSubsystem, shooterSubsystem, aimSubsystem, cpRotatorSubsystem));
    driverController1.start().onTrue(new IdleSubsystemsCommand(transferSubsystem, intakeSubsystem, shooterSubsystem, aimSubsystem, cpRotatorSubsystem));
    //endregion
    //endregion
  }

  /* 
   * If input is below the set deadzone, return 0
   * 
   * If input is above the deadzone, scale the input to a 0-1 range that gets outputted
   */
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
