package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CustomSubsystem;

public class ShooterSubsystem extends SubsystemBase implements CustomSubsystem<ShooterSubsystem.ShooterSubsystemStates> {
    // create transferSubsystem states here
    private ShooterSubsystemStates currentState = ShooterSubsystemStates.IDLE;

    public enum ShooterSubsystemStates {
        IDLE,
        secondState,
        thirdstate
    }

    private final TalonSRX flywheelMotor = new TalonSRX(7);

    @Override
    public void periodic() {
        // This runs every 20ms. Use it to act on the current state.
        switch (currentState) {
            case secondState:
                // Logic to move motors
                break;
            case IDLE:
                // Stop motors
                break;
            default:
                break;
        }
    }

    public Command setFlywheelPowerCommand(double power) {
        return runOnce(() -> {
            setFlywheelPower(power);
        });
    }

    private void setFlywheelPower(double power) {
        flywheelMotor.set(ControlMode.PercentOutput, power);
    }

    @Override
    public ShooterSubsystemStates getState() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getState'");
    }

    @Override
    public void setTargetState(ShooterSubsystemStates state) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setTargetState'");
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
        // set the motor to factory default to start from a known state
        flywheelMotor.configFactoryDefault();
        
        // can reverse motor direction if needed
        flywheelMotor.setInverted(false);
    }
}
