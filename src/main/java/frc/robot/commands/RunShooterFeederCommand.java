// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem.ShooterSubsystemStates;

/** An example command that uses an example subsystem. */
public class RunShooterFeederCommand extends Command {

    ShooterSubsystem shooterSubsystem;
      
    public RunShooterFeederCommand(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        
        addRequirements(shooterSubsystem);
    }
  
    @Override
    public void initialize() {
        shooterSubsystem.setTargetState(ShooterSubsystem.ShooterSubsystemStates.SHOOTING);
    }
  
    @Override
    public void execute() {

    }
  
    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.setTargetState(ShooterSubsystemStates.CHARGED);
    }
  
    @Override
    public boolean isFinished() {
      return false;
    }
}