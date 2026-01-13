package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CustomSubsystem;
import frc.robot.subsystems.PhoenixIDConstants;

public class IntakeSubsystem extends SubsystemBase implements CustomSubsystem<IntakeSubsystem.IntakeSubsystemStates>{
    // create transferSubsystem states here
    private IntakeSubsystemStates currentState = IntakeSubsystemStates.IDLE;
    private IntakeSubsystemStates targetState = IntakeSubsystemStates.IDLE;

    public enum IntakeSubsystemStates {
        INTAKING,
        OUTTAKING,
        IDLE
    }
    private final double baseMotorSpeed = 0.4;

    private final VictorSPX intakeMotor = new VictorSPX(PhoenixIDConstants.INTAKE);
    private final Timer telemetryTimer = new Timer();

    @Override
    public void periodic() {
        outputTelemetry(true);

        if (targetState != currentState) {
            switch (currentState) {
                case IDLE:
                    if (targetState == IntakeSubsystemStates.OUTTAKING) {
                        outtakeIntake();
                        setCurrentState(IntakeSubsystemStates.OUTTAKING);
                    }
                    else if (targetState == IntakeSubsystemStates.INTAKING) {
                        intakeIntake();
                        setCurrentState(IntakeSubsystemStates.INTAKING);
                    }
                    break;
                case OUTTAKING:
                    if (targetState == IntakeSubsystemStates.IDLE) {
                        idleIntake();
                        setCurrentState(IntakeSubsystemStates.IDLE);
                    } 
                    else if (targetState == IntakeSubsystemStates.INTAKING) {
                        intakeIntake();
                        setCurrentState(IntakeSubsystemStates.INTAKING);
                    }
                    break;
                case INTAKING:
                    if (targetState == IntakeSubsystemStates.IDLE) {
                        idleIntake();
                        setCurrentState(IntakeSubsystemStates.IDLE);
                    }
                    else if (targetState == IntakeSubsystemStates.OUTTAKING) {
                        outtakeIntake();
                        setCurrentState(IntakeSubsystemStates.OUTTAKING);
                    }
                    break;
            }
        }
    }

    public Command setMotorVoltageCommand(double power) {
        return runOnce(() -> { setVoltage(power); });
    }

    private void setVoltage(double power) {
        intakeMotor.set(ControlMode.PercentOutput, power);
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

    private void setCurrentState(IntakeSubsystemStates state) {
        currentState = state;
    }

    @Override
    public IntakeSubsystemStates getState() {
        return currentState;
    }

    @Override
    public void setTargetState(IntakeSubsystemStates state) {
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
            System.out.println("Intake Subsystem Motor State: " + currentState);

            telemetryTimer.reset();
        }
    }

    @Override
    public void initializeSubsystem() {
        telemetryTimer.start();

        // set the motor to factory default to start from a known state
        intakeMotor.configFactoryDefault();

        intakeMotor.setNeutralMode(NeutralMode.Brake);
        
        // can reverse motor direction if needed
        intakeMotor.setInverted(false);
    }
}
