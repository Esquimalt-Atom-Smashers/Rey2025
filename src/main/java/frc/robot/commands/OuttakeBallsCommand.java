// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.balltransfer.TransferSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;

/** An example command that uses an example subsystem. */
public class OuttakeBallsCommand extends Command {

    IntakeSubsystem intakeSubsystem;
    TransferSubsystem transferSubsystem;
      
    public OuttakeBallsCommand(IntakeSubsystem intakeSubsystem, TransferSubsystem transferSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.transferSubsystem = transferSubsystem;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(intakeSubsystem, transferSubsystem);
    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        System.out.println("Command running");
        //intakeSubsystem.outtake();
        //Commands.runOnce(() -> intakeSubsystem.outtakeCommand(), intakeSubsystem);
        //transferSubsystem.ejectBallsCommand();

        new RunCommand(() -> intakeSubsystem.outtakeCommand(), intakeSubsystem);
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