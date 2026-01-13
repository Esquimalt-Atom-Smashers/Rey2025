// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;

/** An example command that uses an example subsystem. */
public class ToggleShooterChargingCommand extends Command {

    ShooterSubsystem shooterSubsystem;
      
    public ToggleShooterChargingCommand(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        
        addRequirements(shooterSubsystem);
    }
  
    @Override
    public void initialize() {
        if (shooterSubsystem.getState() == shoo)
        shooterSubsystem.enterShootState();
    }
  
    @Override
    public void execute() {

    }
  
    @Override
    public void end(boolean interrupted) {
        
    }
  
    @Override
    public boolean isFinished() {
      return false;
    }
}