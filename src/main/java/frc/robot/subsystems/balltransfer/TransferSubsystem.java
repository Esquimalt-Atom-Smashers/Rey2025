package frc.robot.subsystems.balltransfer;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CustomSubsystem;
import frc.robot.subsystems.PhoenixIDConstants;

public class TransferSubsystem extends SubsystemBase implements CustomSubsystem<TransferSubsystem.TransferSubsystemStates> {
    // create transferSubsystem states here
    private TransferSubsystemStates currentState = TransferSubsystemStates.IDLE;
    private TransferSubsystemStates targetState = TransferSubsystemStates.IDLE;

    public enum TransferSubsystemStates {
        IDLE,
        TRANSFER,
        EJECT,
        SHUFFLE
    }

    private final double baseMotorSpeed = 0.2;
    
    private final VictorSPX transferMotor = new VictorSPX(PhoenixIDConstants.BALL_TRANSFER);
    private final Timer telemetryTimer = new Timer();

    @Override
    public void periodic() {
        outputTelemetry(false);

        if (targetState != currentState) {
            switch (currentState) {
                case IDLE:
                    if (targetState == TransferSubsystemStates.TRANSFER) {
                        transferBalls();
                        setCurrentState(TransferSubsystemStates.TRANSFER);
                    }
                    else if (targetState == TransferSubsystemStates.EJECT) {
                        ejectBalls();
                        setCurrentState(TransferSubsystemStates.EJECT);
                    }
                    else if (targetState == TransferSubsystemStates.SHUFFLE) {
                        // shuffly
                    }
                    break;
                case TRANSFER:
                    if (targetState == TransferSubsystemStates.IDLE) {
                        idleTransfer();
                        setCurrentState(TransferSubsystemStates.IDLE);
                    }
                    else if (targetState == TransferSubsystemStates.EJECT) {
                        ejectBalls();
                        setCurrentState(TransferSubsystemStates.EJECT);
                    }
                    else if (targetState == TransferSubsystemStates.SHUFFLE) {
                        // shuffle
                    }
                    break;
                case EJECT:
                    if (targetState == TransferSubsystemStates.IDLE) {
                        idleTransfer();
                        setCurrentState(TransferSubsystemStates.IDLE);
                    }
                    else if (targetState == TransferSubsystemStates.TRANSFER) {
                        transferBalls();
                        setCurrentState(TransferSubsystemStates.TRANSFER);
                    }
                    else if (targetState == TransferSubsystemStates.SHUFFLE) {
                        // shuffle
                    }
                    break;
                case SHUFFLE:
                    // too lazy to add rn
                    break;
            }
        }
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

    private void idleTransfer() {
        setVoltage(0);
    }

    private void ejectBalls() {
        setVoltage(-baseMotorSpeed);
    }

    private void transferBalls() {
        setVoltage(baseMotorSpeed);
    }

    private void setCurrentState(TransferSubsystemStates state) {
        currentState = state;
    }

    @Override
    public void setTargetState(TransferSubsystemStates state) {
        targetState = state;
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
