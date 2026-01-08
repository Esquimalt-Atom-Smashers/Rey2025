package frc.robot.subsystems.balltransfer;

import com.ctre.phoenix.motion.SetValueMotionProfile;
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
        EJECT_BALLS,
        SHUFFLE_BALLS,
        MANUAL_OVERRIDE
    }
    
    private final VictorSPX transferMotor = new VictorSPX(11);
    private final Timer telemetryTimer = new Timer();

    @Override
    public void periodic() {
        // This runs every 20ms. Use it to act on the current state.
        switch (currentState) {
            case TRANSFER_BALLS:
                setVoltage(0.2);
                break;
            case EJECT_BALLS:
                setVoltage(-0.2);
            case SHUFFLE_BALLS:
                // shuffle balls
                break;
            case IDLE:
                setVoltage(0);
                break;
            case MANUAL_OVERRIDE:
                break;
            default:
                break;
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

    public Command transferBalls() {
        return runOnce((() -> { setTargetState(TransferSubsystemStates.TRANSFER_BALLS); } ));
    }

    public Command ejectBalls() {
        return runOnce((() -> { setTargetState(TransferSubsystemStates.EJECT_BALLS); } ));
    }

    public Command idle() {
        return runOnce((() -> { setTargetState(TransferSubsystemStates.IDLE); } ));
    }

    public Command manualOveride(double voltage) {
        return runOnce((() -> { 
            setTargetState(TransferSubsystemStates.MANUAL_OVERRIDE);
            setVoltage(voltage);
         } ));
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
            System.out.println("Transfer Subsystem");

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
