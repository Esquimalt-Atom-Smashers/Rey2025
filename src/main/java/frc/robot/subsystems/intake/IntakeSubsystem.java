package frc.robot.subsystems.intake;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CustomSubsystem;


public class IntakeSubsystem extends SubsystemBase implements CustomSubsystem<IntakeSubsystem.IntakeSubsystemStates> {
    // create transferSubsystem states here
    private IntakeSubsystemStates currentState = IntakeSubsystemStates.IDLE;


    public enum IntakeSubsystemStates {
        IDLE,
        INTAKE,
        OUTTAKE
    }


    private final VictorSPX transferMotor = new VictorSPX(10);
    private final Timer telemetryTimer = new Timer();


    public IntakeSubsystem (){
        //telemetryTimer.start();
    }


    @Override
    public void periodic() {
        if (telemetryTimer.hasElapsed(1)) {
            outputTelemetry(true);
            telemetryTimer.reset();
        }
    }  


    public Command setMotorVoltageCommand(double power) {
        return runOnce(() -> { setVoltage(power);});
    }


    private void setVoltage(double power) {
        transferMotor.set(ControlMode.PercentOutput, power);
    }


    @Override
    public IntakeSubsystemStates getState() {
       return currentState;
    }


    @Override
   public void setTargetState(IntakeSubsystemStates state) {
    currentState = state;
   }


    public Command idle() {
        return runOnce((() -> {
            setTargetState(IntakeSubsystemStates.IDLE);
            setVoltage(0);
        }));
    }


    public Command intakeBalls() {
        return runOnce((() -> {
            setTargetState(IntakeSubsystemStates.INTAKE);
            setVoltage(0.2);
        }));
    }


    public Command outtakeBalls() {
        return runOnce((() -> {
            setTargetState(IntakeSubsystemStates.OUTTAKE);
            setVoltage(-0.2);
        }));
    }


    @Override
    public void shutdownSubsystem() {
        setVoltage(0);
    }


    @Override
    public void resetSubsystem() {
        currentState = IntakeSubsystemStates.IDLE;
        setVoltage(0);
    }


    @Override
    public void outputTelemetry(boolean enableTelemetry) {
        System.out.println("test Telemetry for the Intake SS");
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


