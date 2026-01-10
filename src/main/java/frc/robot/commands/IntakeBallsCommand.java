// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.balltransfer.TransferSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;

/** An example command that uses an example subsystem. */
public class IntakeBallsCommand extends Command {

    IntakeSubsystem intakeSubsystem;
    TransferSubsystem transferSubsystem;
      
    public IntakeBallsCommand(IntakeSubsystem intakeSubsystem, TransferSubsystem transferSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.transferSubsystem = transferSubsystem;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(intakeSubsystem, transferSubsystem);
    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        //intakeSubsystem.intakeCommand();
        //transferSubsystem.transferBallsCommand();
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.idleCommand();
        transferSubsystem.idleCommand();
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }
}