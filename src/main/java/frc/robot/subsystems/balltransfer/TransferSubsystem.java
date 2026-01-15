package frc.robot.subsystems.balltransfer;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CustomSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem.IntakeSubsystemStates;

public class TransferSubsystem extends SubsystemBase implements CustomSubsystem<TransferSubsystem.TransferSubsystemStates> {
    // create transferSubsystem states here
    private TransferSubsystemStates currentState = TransferSubsystemStates.IDLE;
    
    private final VictorSPX ballTransferMotor = new VictorSPX(11);
    public double intakeMotorPower = -0.5;
    public double outakeMotorPower = 0.5;
    private Timer telemetryTimer = new Timer();

    /*public void changeBallTransferPower (double newPower) {
        intakeMotorPower = newPower;
        outakeMotorPower = -newPower;
    } */

    public void setPower(double power) {
        ballTransferMotor.set(ControlMode.PercentOutput, power);
    }
    
    public enum TransferSubsystemStates {
        IDLE,
        TRANSFER_BALLS,
        SHUFFLE_BALLS,
        OUTAKING
    }

    public Command setTransferStateCommand(TransferSubsystemStates state) {
    return runOnce(() -> { setTargetState(state); });
    }

    @Override
    public void periodic() {
        // This runs every 20ms. Use it to act on the current state.
        outputTelemetry(true);
        
        switch (currentState) {
            case TRANSFER_BALLS:
                setPower(intakeMotorPower);
                break;
            case OUTAKING:
                setPower(outakeMotorPower);
                break;
            case IDLE:
                setPower(0);
                break;
            default:
                break;
        }
    }

    @Override
    public TransferSubsystemStates getState() {
        return currentState;
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
        if (!enableTelemetry){
            return;
        }
        if (telemetryTimer.get() > 1){
            System.out.println(getState());
            telemetryTimer.reset();
        }
    }

    @Override
    public void initializeSubsystem() {
        telemetryTimer.start();
        ballTransferMotor.configFactoryDefault();
        ballTransferMotor.setInverted(false);
    }

}
