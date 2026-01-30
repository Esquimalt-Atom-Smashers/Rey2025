// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.balltransfer.TransferSubsystem;
import frc.robot.subsystems.balltransfer.TransferSubsystem.TransferSubsystemStates;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem.IntakeSubsystemStates;
import edu.wpi.first.wpilibj.Timer;

/** An example command that uses an example subsystem. */
public class ShuffleBallsCommand extends Command {

    IntakeSubsystem intakeSubsystem;
    TransferSubsystem transferSubsystem;
    Timer timer = new Timer();

    final double SHUFFLE_TIME = 0.1;
      
    public ShuffleBallsCommand(IntakeSubsystem intakeSubsystem, TransferSubsystem transferSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.transferSubsystem = transferSubsystem;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(intakeSubsystem, transferSubsystem);
    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        timer.start();
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (timer.get() < SHUFFLE_TIME) {
            intakeSubsystem.setTargetState(IntakeSubsystemStates.INTAKING);
            transferSubsystem.setTargetState(TransferSubsystemStates.TRANSFER);
        } else if (timer.get() > (SHUFFLE_TIME * 2)) {
            timer.reset();
        } else if (timer.get() > SHUFFLE_TIME) {
            intakeSubsystem.setTargetState(IntakeSubsystemStates.OUTTAKING);
            transferSubsystem.setTargetState(TransferSubsystemStates.EJECT);
        }
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.setTargetState(IntakeSubsystem.IntakeSubsystemStates.IDLE);
        transferSubsystem.setTargetState(TransferSubsystemStates.IDLE);
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }
}