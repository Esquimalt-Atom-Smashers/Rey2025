package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.balltransfer.TransferSubsystem;
import frc.robot.subsystems.balltransfer.TransferSubsystem.TransferSubsystemStates;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem.IntakeSubsystemStates;

public class outakeCommand extends ParallelCommandGroup {
    
    private final IntakeSubsystem intakeSubsystem;
    private final TransferSubsystem transferSubsystem;

    public outakeCommand( 
        //these are the parameters of the constructor
        IntakeSubsystem intakeSubsystem,
        TransferSubsystem transferSubsystem
        )
        //this is what the constructor actually does
        {
        this.intakeSubsystem = intakeSubsystem;
        this.transferSubsystem = transferSubsystem;

        this.addCommands(
            intakeSubsystem.setIntakeStateCommand(IntakeSubsystemStates.OUTAKING),
            transferSubsystem.setTransferStateCommand(TransferSubsystemStates.OUTAKING)
        );
        }
        
}
