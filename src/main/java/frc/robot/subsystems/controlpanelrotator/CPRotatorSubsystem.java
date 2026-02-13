package frc.robot.subsystems.controlpanelrotator;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CustomSubsystem;
import frc.robot.subsystems.PhoenixIDConstants;

public class CPRotatorSubsystem extends SubsystemBase implements CustomSubsystem<CPRotatorSubsystem.CPRotatorSubsystemStates>{
    // create transferSubsystem states here
    private CPRotatorSubsystemStates currentState = CPRotatorSubsystemStates.IDLE;
    private CPRotatorSubsystemStates targetState = CPRotatorSubsystemStates.IDLE;

    public enum CPRotatorSubsystemStates {
        INTAKING,
        OUTTAKING,
        IDLE
    }
    private final double baseMotorSpeed = 0.8;

    private final VictorSPX cpRotatorMotor = new VictorSPX(PhoenixIDConstants.GRENADE);
    private final Timer telemetryTimer = new Timer();

    @Override
    public void periodic() {
        outputTelemetry(false);

        if (targetState != currentState) {
            switch (currentState) {
                case IDLE -> handleIDLE();
                case OUTTAKING -> handleOUTTAKING();
                case INTAKING -> handleINTAKING();
            }
        }
    }

    //region Handle States
    private void handleIDLE() {
        if (targetState == CPRotatorSubsystemStates.OUTTAKING) {
            outtakeIntake();
            setCurrentState(CPRotatorSubsystemStates.OUTTAKING);
        }
        else if (targetState == CPRotatorSubsystemStates.INTAKING) {
            intakeIntake();
            setCurrentState(CPRotatorSubsystemStates.INTAKING);
        }
    }

    private void handleOUTTAKING() {
        if (targetState == CPRotatorSubsystemStates.IDLE) {
            idleIntake();
            setCurrentState(CPRotatorSubsystemStates.IDLE);
        } 
        else if (targetState == CPRotatorSubsystemStates.INTAKING) {
            intakeIntake();
            setCurrentState(CPRotatorSubsystemStates.INTAKING);
        }
    }

    private void handleINTAKING() {
        if (targetState == CPRotatorSubsystemStates.IDLE) {
            idleIntake();
            setCurrentState(CPRotatorSubsystemStates.IDLE);
        }
        else if (targetState == CPRotatorSubsystemStates.OUTTAKING) {
            outtakeIntake();
            setCurrentState(CPRotatorSubsystemStates.OUTTAKING);
        }
    }

    //endregion
    public Command setMotorVoltageCommand(double power) {
        return runOnce(() -> { setVoltage(power); });
    }

    public void setVoltage(double power) {
        cpRotatorMotor.set(ControlMode.PercentOutput, power);
    }

    private void idleIntake() {
        setVoltage(0);
    }

    private void outtakeIntake() {
        setVoltage(-baseMotorSpeed);
    }

    private void intakeIntake() {
        setVoltage(baseMotorSpeed);
    }

    private void setCurrentState(CPRotatorSubsystemStates state) {
        currentState = state;
    }

    @Override
    public CPRotatorSubsystemStates getState() {
        return currentState;
    }

    @Override
    public void setTargetState(CPRotatorSubsystemStates state) {
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
            System.out.println("Control Panel Rotator: " + currentState);

            telemetryTimer.reset();
        }
    }

    @Override
    public void initializeSubsystem() {
        telemetryTimer.start();

        // set the motor to factory default to start from a known state
        cpRotatorMotor.configFactoryDefault();

        cpRotatorMotor.setNeutralMode(NeutralMode.Brake);
        
        // can reverse motor direction if needed
        cpRotatorMotor.setInverted(false);
    }
}
