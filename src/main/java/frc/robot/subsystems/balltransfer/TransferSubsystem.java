package frc.robot.subsystems.balltransfer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CustomSubsystem;

public class TransferSubsystem extends SubsystemBase implements CustomSubsystem<TransferSubsystem.TransferSubsystemStates> {
    // create transferSubsystem states here
    private TransferSubsystemStates currentState = TransferSubsystemStates.IDLE;

    public enum TransferSubsystemStates {
        IDLE,
        TRANSFER_BALLS,
        SHUFFLE_BALLS
    }
    
    @Override
    public void periodic() {
        // This runs every 20ms. Use it to act on the current state.
        switch (currentState) {
            case TRANSFER_BALLS:
                // Logic to move motors
                break;
            case IDLE:
                // Stop motors
                break;
            default:
                break;
        }
    }

    @Override
    public TransferSubsystemStates getState() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getState'");
    }

    @Override
    public void setTargetState(TransferSubsystemStates state) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setTargetState'");
    }

    @Override
    public void shutdownSubsystem() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'shutdownSubsystem'");
    }

    @Override
    public void resetSubsystem() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'resetSubsystem'");
    }

    @Override
    public void outputTelemetry(boolean enableTelemetry) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'outputTelemetry'");
    }

    @Override
    public void initializeSubsystem() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'initializeSubsystem'");
    }

}
