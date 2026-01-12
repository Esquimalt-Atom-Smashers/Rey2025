package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CustomSubsystem;
import frc.robot.subsystems.PhoenixIDConstants;

public class IntakeSubsystem extends SubsystemBase implements CustomSubsystem<IntakeSubsystem.IntakeSubsystemStates> {
    // create transferSubsystem states here
    private IntakeSubsystemStates currentState = IntakeSubsystemStates.IDLE;
    private final VictorSPX intakeMotor = new VictorSPX(PhoenixIDConstants.INTAKE);
    private Timer telemetryTimer = new Timer();
    
    public enum IntakeSubsystemStates {
        IDLE,
        INTAKING,
        OUTAKING
    }
   
    private void setVoltage(double power) {
        intakeMotor.set(ControlMode.PercentOutput, power);
    }

    public void setTargetState(IntakeSubsystemStates state){
        currentState = state;
    }

    public Command setIntakePower(double power) {
        return runOnce(() -> {setVoltage(power);});
    }

    public Command intakeCommand(double power){
        setTargetState(IntakeSubsystemStates.INTAKING);
        return setIntakePower(power);
    }

    public Command outtakeCommand (double power){
        setTargetState(IntakeSubsystemStates.OUTAKING);
        return setIntakePower(-power);
    }
    
    public Command setIntakeStateCommand(IntakeSubsystemStates state) {
        return runOnce(() -> { setTargetState(state); });
    }
   
    public Command idleCommand() {
        return runOnce(() -> {shutdownSubsystem();});
    }
    @Override
    
    public void periodic() {
        // This runs every 20ms. Use it to act on the current state.
        outputTelemetry(true);
        switch (currentState) {
            case INTAKING:
                setVoltage(0.5);
                break;
            case IDLE:
                setVoltage(0);
                break;
            case OUTAKING:
                setVoltage(-0.5);
                break;
            default:
                break;
        }
    }

    @Override
    public IntakeSubsystemStates getState() {
        return currentState;
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
        if (!enableTelemetry){
            return;
        }

        if (telemetryTimer.get() > 1) {
            System.out.println("Intake Motor current state: " + currentState);
        
            telemetryTimer.reset();
        }
        
    }

    @Override
    public void initializeSubsystem() {
        
        telemetryTimer.start();
        intakeMotor.configFactoryDefault();
        intakeMotor.setInverted(false);
        
    }
}
