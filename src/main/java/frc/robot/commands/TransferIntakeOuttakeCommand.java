package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.balltransfer.TransferSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class TransferIntakeOuttakeCommand extends ParallelCommandGroup{
    
    public TransferIntakeOuttakeCommand(TransferSubsystem transferSubsystem, IntakeSubsystem intakeSubsystem) {
        this.addCommands(
            transferSubsystem.ejectBallsCommand(),
            intakeSubsystem.outtakeCommand()
        );
    }
}
