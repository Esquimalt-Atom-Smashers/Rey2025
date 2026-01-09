package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.balltransfer.TransferSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class TransferIntakeIdleCommand extends ParallelCommandGroup{
    
    public TransferIntakeIdleCommand(TransferSubsystem transferSubsystem, IntakeSubsystem intakeSubsystem) {
        this.addCommands(
            transferSubsystem.idleCommand(),
            intakeSubsystem.idleCommand()
        );
    }
}
