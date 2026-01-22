// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.AimSubsystem;
import frc.robot.subsystems.shooter.AimSubsystem.AimingSubsystemStates;

/** An example command that uses an example subsystem. */
public class ToggleAimingHoodCommand extends Command {

    AimSubsystem aimSubsystem;
      
    public ToggleAimingHoodCommand(AimSubsystem shooterSubsystem) {
        this.aimSubsystem = shooterSubsystem;
        
        addRequirements(shooterSubsystem);
    }
  
    @Override
    public void initialize() {
        if (aimSubsystem.getState() != AimingSubsystemStates.AIMED)
            aimSubsystem.setTargetState(AimSubsystem.AimingSubsystemStates.AIMED);
        else 
            aimSubsystem.setTargetState(AimingSubsystemStates.IDLE);
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