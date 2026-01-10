package frc.robot.subsystems.shooter;

import java.rmi.server.SocketSecurityException;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CustomSubsystem;
import frc.robot.subsystems.PhoenixIDConstants;
import edu.wpi.first.wpilibj.Timer;

public class ShooterSubsystem extends SubsystemBase implements CustomSubsystem<ShooterSubsystem.ShooterSubsystemStates> {
    // create transferSubsystem states here
    private ShooterSubsystemStates currentState = ShooterSubsystemStates.idle;

    public enum ShooterSubsystemStates {
        charging,
        feeding,
        idle
    }

    private final Timer telemetryTimer = new Timer();
    private final TalonSRX flywheelMotor = new TalonSRX(PhoenixIDConstants.FLYWHEEL);
    private final Encoder flywheelEncoder = new Encoder(1, 2);
    private final PIDController flywheelVelocityController = new PIDController(0.5, 0.0, 0.0);

    private final VictorSPX feederMotor = new VictorSPX(PhoenixIDConstants.SHOOTER_FEEDER);

    public final double baseFlywheelVelocity = 100;
    public final double slowFlywheelVelocity = 80;
    public final double fastFlywheelVelocity = 120;
    private double currentFlywheelVelocity = baseFlywheelVelocity;

    private final double feedingPower = 1;

    @Override
    public void periodic() {
        outputTelemetry(true);
    }

    public Command setFlywheelVelocityCommand(double velocity) {
        return runOnce(() -> {
            setFlywheelVelocity(velocity);
        });
    }

    public Command setFeederPowerCommand(double power) {
        return runOnce(() -> {
            setFeederPower(power);
        });
    }

    public Command runFlywheelAtSpeedCommand(double targetVelocity) {
        return run(() -> {
            runFlywheelAtSpeed(targetVelocity);
        });
    }

    public Command idleCommand() {
        return runOnce(() -> {
            setFlywheelVelocity(0);
            setFeederPower(0);
            setTargetState(ShooterSubsystemStates.idle);
        });
    }

    public Command feedingCommand() {
        return runOnce(() -> {
            setFlywheelVelocity(currentFlywheelVelocity);
            setFeederPower(feedingPower);
            setTargetState(ShooterSubsystemStates.feeding);
        });
    }

    public Command chargingCommand() {
        return runOnce(() -> {
            setFlywheelVelocity(currentFlywheelVelocity);
            setFeederPower(0);
            setTargetState(ShooterSubsystemStates.charging);
        });
    }

    public Command setCurrentFlywheelVelocity(double velocity) {
        return runOnce(() -> { currentFlywheelVelocity = velocity; });
    }

    private void setFlywheelVelocity(double velocity) {
        flywheelMotor.set(ControlMode.Velocity, velocity);
    }

    private void setFeederPower(double power) {
        feederMotor.set(ControlMode.PercentOutput, power);
    }

    private void runFlywheelAtSpeed(double targetVelocity) {
        double output = flywheelVelocityController.calculate(flywheelEncoder.getRate(), targetVelocity);
        flywheelMotor.set(ControlMode.PercentOutput, output);
    }

    @Override
    public ShooterSubsystemStates getState() {
        return currentState;
    }

    @Override
    public void setTargetState(ShooterSubsystemStates state) {
        currentState = state;
    }

    @Override
    public void shutdownSubsystem() {
        setFeederPower(0);
        setFlywheelVelocity(0);
    }

    @Override
    public void resetSubsystem() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'resetSubsystem'");
    }

    @Override
    public void outputTelemetry(boolean enableTelemetry) {
        if (telemetryTimer.hasElapsed(1)) {
            
            System.out.println("Current Shooter State: " + currentState);

            telemetryTimer.reset();
        }
    }

    @Override
    public void initializeSubsystem() {
        flywheelMotor.configFactoryDefault();
        feederMotor.configFactoryDefault();
        
        flywheelMotor.setInverted(false);
        feederMotor.setInverted(false);

        telemetryTimer.start();
    }
}
