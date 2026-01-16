// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.balltransfer.TransferSubsystem;
import frc.robot.subsystems.balltransfer.TransferSubsystem.TransferSubsystemStates;
import frc.robot.subsystems.drivebase.DrivebaseSubsystem;

/** An example command that uses an example subsystem. */
public class DriveSlowModeCommand extends Command {

    DrivebaseSubsystem drivebaseSubsystem;

    public DriveSlowModeCommand(DrivebaseSubsystem drivebaseSubsystem) {
        this.drivebaseSubsystem = drivebaseSubsystem;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drivebaseSubsystem);
    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        drivebaseSubsystem.setSlowSpeed();
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        drivebaseSubsystem.setDefaultSpeed();
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }
}