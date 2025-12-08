// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.CustomSubsystem;
import frc.robot.subsystems.balltransfer.TransferSubsystem;
import frc.robot.subsystems.controlpanelrotator.CPRotatorSubsystem;
import frc.robot.subsystems.drivebase.DrivebaseSubsystem;
import frc.robot.subsystems.hang.HangingSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.ledlights.BlinkinSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class RobotContainer{
  private IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private TransferSubsystem transferSubsystem = new TransferSubsystem();
  private ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private DrivebaseSubsystem drivebaseSubsystem = new DrivebaseSubsystem();
  private HangingSubsystem hangingSubsystem = new HangingSubsystem();
  private BlinkinSubsystem ledSubsystem = new BlinkinSubsystem();
  private CPRotatorSubsystem cpRotatorSubsystem = new CPRotatorSubsystem();

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {}

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
  public void outputTelemetry() {
    intakeSubsystem.outputTelemetry();
    transferSubsystem.outputTelemetry();
    shooterSubsystem.outputTelemetry();
    drivebaseSubsystem.outputTelemetry();
    hangingSubsystem.outputTelemetry();
    ledSubsystem.outputTelemetry();
    cpRotatorSubsystem.outputTelemetry();    
  }
}
