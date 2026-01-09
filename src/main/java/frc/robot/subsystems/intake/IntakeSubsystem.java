package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CustomSubsystem;

public class IntakeSubsystem extends SubsystemBase implements CustomSubsystem<IntakeSubsystem.IntakeMotorState>{
    // create transferSubsystem states here
    private IntakeMotorState currentMotorState = IntakeMotorState.idle;

    public enum IntakeMotorState {
        intaking,
        outtaking,
        idle,
        manualOverride
    }
    private final double baseMotorSpeed = 0.4;

    private final VictorSPX transferMotor = new VictorSPX(10);
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

    private void setPneumaticControl(double power) {

    }

    public Command manualOverrideCommand(double power) {
        return runOnce(() -> {
            setTargetState(IntakeMotorState.manualOverride);
            setVoltage(power);
        });
    }

    public Command idleCommand() {
        return runOnce(() -> {
            setTargetState(IntakeMotorState.idle);
            setVoltage(0);
        });
    }

    public Command intakeCommand() {
        return runOnce(() -> {
            setTargetState(IntakeMotorState.intaking);
            setVoltage(baseMotorSpeed);
        });
    }

    public Command outtakeCommand() {
        return runOnce(() -> {
            setTargetState(IntakeMotorState.outtaking);
            setVoltage(-baseMotorSpeed);
        });
    }

    @Override
    public IntakeMotorState getState() {
        return currentMotorState;
    }

    @Override
    public void setTargetState(IntakeMotorState state) {
        currentMotorState = state;
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
            System.out.println("Intake Subsystem Motor State: " + currentMotorState);

            telemetryTimer.reset();
        }
    }

    @Override
    public void initializeSubsystem() {
        telemetryTimer.start();

        // set the motor to factory default to start from a known state
        transferMotor.configFactoryDefault();
        
        // can reverse motor direction if needed
        transferMotor.setInverted(false);
    }
}
