package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.balltransfer.TransferSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class TransferIntakeIntakeCommand extends ParallelCommandGroup{
    
    public TransferIntakeIntakeCommand(TransferSubsystem transferSubsystem, IntakeSubsystem intakeSubsystem) {
        this.addCommands(
            transferSubsystem.transferBallsCommand(),
            intakeSubsystem.intakeCommand()
        );
    }
}
