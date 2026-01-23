package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.balltransfer.TransferSubsystem;
import frc.robot.subsystems.balltransfer.TransferSubsystem.TransferSubsystemStates;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem.IntakeSubsystemStates;
import frc.robot.subsystems.shooter.AimSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooter.AimSubsystem.AimingSubsystemStates;
import frc.robot.subsystems.shooter.ShooterSubsystem.ShooterSubsystemStates;

public class IdleSubsystemsCommand extends ParallelCommandGroup{
    
    public IdleSubsystemsCommand(TransferSubsystem transferSubsystem, IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem, AimSubsystem aimSubsystem) {
        this.addCommands(
            new InstantCommand(() -> { transferSubsystem.setTargetState(TransferSubsystemStates.IDLE); }),
            new InstantCommand(() -> { intakeSubsystem.setTargetState(IntakeSubsystemStates.IDLE); }),
            new InstantCommand(() -> { shooterSubsystem.setTargetState(ShooterSubsystemStates.IDLE); }),
            new InstantCommand(() -> { aimSubsystem.setTargetState(AimingSubsystemStates.IDLE); })
        );
    }
}
