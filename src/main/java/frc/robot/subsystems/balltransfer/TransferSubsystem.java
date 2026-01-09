package frc.robot.subsystems.balltransfer;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CustomSubsystem;

public class TransferSubsystem extends SubsystemBase implements CustomSubsystem<TransferSubsystem.TransferSubsystemStates> {
    // create transferSubsystem states here
    private TransferSubsystemStates currentState = TransferSubsystemStates.idle;

    public enum TransferSubsystemStates {
        idle,
        transferBalls,
        ejectBalls,
        shuffleBalls,
        manualOverride
    }

    private final double baseMotorSpeed = 0.2;
    
    private final VictorSPX transferMotor = new VictorSPX(11);
    private final Timer telemetryTimer = new Timer();

    @Override
    public void periodic() {
        outputTelemetry(true);
    }

    public Command setMotorVoltageCommand(double power) {
        return runOnce(() -> { setVoltage(power); });
    }

    private void setVoltage(double power) {
        transferMotor.set(ControlMode.PercentOutput, power);
    }

    @Override
    public TransferSubsystemStates getState() {
        return currentState;
    }

    public Command transferBalls() {
        return runOnce(() -> { 
            setTargetState(TransferSubsystemStates.transferBalls); 
            setVoltage(baseMotorSpeed);
        } );
    }

    public Command ejectBalls() {
        return runOnce(() -> { 
            setTargetState(TransferSubsystemStates.ejectBalls);
            setVoltage(-baseMotorSpeed);
         } );
    }

    public Command idle() {
        return runOnce(() -> { 
            setTargetState(TransferSubsystemStates.idle);
            setVoltage(0);
         } );
    }

    public Command manualOveride(double voltage) {
        return runOnce(() -> { 
            setTargetState(TransferSubsystemStates.manualOverride);
            setVoltage(voltage);
         } );
    }

    @Override
    public void setTargetState(TransferSubsystemStates state) {
        currentState = state;
    }

    @Override
    public void shutdownSubsystem() {
        setVoltage(0);
    }

    @Override
    public void resetSubsystem() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'resetSubsystem'");
    }

    @Override
    public void outputTelemetry(boolean enableTelemetry) {
        if (!enableTelemetry)
            return;

        if (telemetryTimer.hasElapsed(1)) {
            System.out.println("Transfer Subsystem State: " + currentState);

            telemetryTimer.reset();
        }
    }

    @Override
    public void initializeSubsystem() {
        telemetryTimer.start();

        // set the motor to factory default to start from a known state
        transferMotor.configFactoryDefault();
        
        // can reverse motor direction if needed
        transferMotor.setInverted(true);
    }

}
