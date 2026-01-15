// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.idleCommand;
import frc.robot.commands.intakeCommand;
import frc.robot.commands.outakeCommand;
import frc.robot.subsystems.balltransfer.TransferSubsystem;
import frc.robot.subsystems.controlpanelrotator.CPRotatorSubsystem;
import frc.robot.subsystems.drivebase.DrivebaseSubsystem;
import frc.robot.subsystems.hang.HangingSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem.IntakeSubsystemStates;
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
  private final CommandXboxController xboxController = new CommandXboxController(0);
  
  public RobotContainer() {
    //allows you to easily change the power
    //transferSubsystem.changeBallTransferPower(0.5);
    
    configureBindings();
    
    intakeSubsystem.initializeSubsystem();
  }
  
  private void configureBindings() {

      drivebaseSubsystem.setDefaultCommand(
        
        drivebaseSubsystem.drive(
          () -> applyDeadzone(xboxController.getRightY(), 0.2),
          () -> applyDeadzone(xboxController.getRightX(), 0.2)
          )
      ); 
      xboxController.leftBumper().onTrue(new intakeCommand(intakeSubsystem,transferSubsystem));
      xboxController.rightBumper().onTrue(new outakeCommand(intakeSubsystem, transferSubsystem));
      xboxController.x().onTrue(new idleCommand(intakeSubsystem, transferSubsystem));
      xboxController.leftTrigger().onTrue(drivebaseSubsystem.slowMode());
  } 
  
  public double applyDeadzone(double value, double deadzone){
    if (Math.abs(value) < deadzone) {
      return 0.0;
    } else {
      double sign = Math.signum(value);
      double scaledValue = (Math.abs(value) - deadzone) / (1.0 - deadzone);
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
  }
  
}
