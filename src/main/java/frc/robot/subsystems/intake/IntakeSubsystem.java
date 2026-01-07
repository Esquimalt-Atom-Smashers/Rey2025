package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CustomSubsystem;

public class IntakeSubsystem extends SubsystemBase implements CustomSubsystem<IntakeSubsystem.IntakeSubsystemStates> {
    // create transferSubsystem states here
    private IntakeSubsystemStates currentState = IntakeSubsystemStates.IDLE;

    public enum IntakeSubsystemStates {
        IDLE,
        secondState,
        thirdstate
    }

    @Override
    public void periodic() {
        // This runs every 20ms. Use it to act on the current state.
        switch (currentState) {
            case secondState:
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
    public IntakeSubsystemStates getState() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getState'");
    }

    @Override
    public void setTargetState(IntakeSubsystemStates state) {
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
