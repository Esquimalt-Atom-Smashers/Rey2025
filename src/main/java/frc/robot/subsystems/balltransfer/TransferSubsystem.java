package frc.robot.subsystems.balltransfer;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
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
    
    private final VictorSPX transferMotor = new VictorSPX(11);
    private Timer printTimer = new Timer();

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

    public TransferSubsystem() {
        // set the motor to factory default to start from a known state
        transferMotor.configFactoryDefault();
        
        // can reverse motor direction if needed
        transferMotor.setInverted(true);
        printTimer.start();
    }

    public Command setMotorVoltageCommand(double power) {
        return runOnce(() -> { setVoltage(power); });
    }

    private void setVoltage(double power) {
        transferMotor.set(ControlMode.PercentOutput, power);
    }

    @Override
    public TransferSubsystemStates getState() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getState'");
    }

    @Override
    public void setTargetState(TransferSubsystemStates state) {
        currentState = state;
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
