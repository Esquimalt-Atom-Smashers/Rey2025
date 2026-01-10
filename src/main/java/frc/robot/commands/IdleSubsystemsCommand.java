package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.balltransfer.TransferSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class IdleSubsystemsCommand extends ParallelCommandGroup{
    
    public IdleSubsystemsCommand(TransferSubsystem transferSubsystem, IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem) {
        this.addCommands(
            transferSubsystem.idleCommand(),
            intakeSubsystem.idleCommand(),
            shooterSubsystem.idleCommand()
        );
    }
}
