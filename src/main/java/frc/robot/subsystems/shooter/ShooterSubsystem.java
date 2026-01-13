package frc.robot.subsystems.shooter;

import java.rmi.server.SocketSecurityException;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
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
    private final PIDController flywheelVelocityController = new PIDController(0.5, 0.0, 0.0);

    private final VictorSPX feederMotor = new VictorSPX(PhoenixIDConstants.SHOOTER_FEEDER);

    public final double baseFlywheelVelocity = 600;
    public final double slowFlywheelVelocity = 300;
    public final double fastFlywheelVelocity = 1000;
    private double currentFlywheelVelocity = baseFlywheelVelocity;

    // Flywheel velocity setup
    double maxRPM = 2000;
    double encoderCPR = 4096;
    double maxTalonUnits = maxRPM * encoderCPR / 600;

    private final double feedingPower = 0.2;

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

    public Command setCurrentFlywheelVelocity(double velocity) {
        return runOnce(() -> { currentFlywheelVelocity = velocity; });
    }

    public void charging() {
        System.out.println("Setting motor to charging");
        setFlywheelVelocity(currentFlywheelVelocity);
        setFeederPower(0);
        setTargetState(ShooterSubsystemStates.charging);
    }

    public void feeding() {
        setFlywheelVelocity(currentFlywheelVelocity);
        setFeederPower(feedingPower);
        setTargetState(ShooterSubsystemStates.feeding);
    }

    public void idle() {
        System.out.println("Setting motor to idle");
        setFlywheelVelocity(0);
        setFeederPower(0);
        setTargetState(ShooterSubsystemStates.idle);
    }

    private void setFlywheelVelocity(double velocity) {
        double targetVelocity = velocity * encoderCPR / 600;
        flywheelMotor.set(ControlMode.Velocity, targetVelocity);
    }

    private void setFeederPower(double power) {
        feederMotor.set(ControlMode.PercentOutput, power);
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
            System.out.println("Motor pos: " + flywheelMotor.getSelectedSensorPosition());

            telemetryTimer.reset();
        }
    }

    @Override
    public void initializeSubsystem() {
        flywheelMotor.configFactoryDefault();
        feederMotor.configFactoryDefault();
        
        flywheelMotor.setInverted(true);
        feederMotor.setInverted(true);

        telemetryTimer.start();

        flywheelMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 30);

        flywheelMotor.config_kP(0, 0.1);
        flywheelMotor.config_kI(0, 0.0);
        flywheelMotor.config_kD(0, 0.0);

        double kF = 1023 / maxTalonUnits;
        flywheelMotor.config_kF(0, kF);
    }
}
