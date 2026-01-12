package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.balltransfer.TransferSubsystem;
import frc.robot.subsystems.balltransfer.TransferSubsystem.TransferSubsystemStates;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem.IntakeSubsystemStates;

public class idleCommand extends ParallelCommandGroup {
    
    private final IntakeSubsystem intakeSubsystem;
    private final TransferSubsystem transferSubsystem;

    public idleCommand( 
        //these are the parameters of the constructor
        IntakeSubsystem intakeSubsystem,
        TransferSubsystem transferSubsystem
        )
        //this is what the constructor actually does
        {
        this.intakeSubsystem = intakeSubsystem;
        this.transferSubsystem = transferSubsystem;

        this.addCommands(
            intakeSubsystem.setIntakeStateCommand(IntakeSubsystemStates.IDLE),
            transferSubsystem.setTransferStateCommand(TransferSubsystemStates.IDLE)
        );
        }
        
}
