package frc.robot.subsystems.shooter;

import java.lang.annotation.Target;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CustomSubsystem;
import frc.robot.subsystems.PhoenixIDConstants;

public class ShooterSubsystem extends SubsystemBase implements CustomSubsystem<ShooterSubsystem.ShooterSubsystemStates> {
    private ShooterSubsystemStates currentState = ShooterSubsystemStates.IDLE;
    private ShooterSubsystemStates targetState = ShooterSubsystemStates.IDLE;

    public enum ShooterSubsystemStates {
        CHARGING,
        CHARGED,
        SHOOTING,
        IDLE
    }

    private final Timer telemetryTimer = new Timer();
    private final TalonSRX flywheelMotor = new TalonSRX(PhoenixIDConstants.FLYWHEEL);

    private final VictorSPX feederMotor = new VictorSPX(PhoenixIDConstants.SHOOTER_FEEDER);

    public static final double FLYWHEEL_CLOSED_LOOP_ERROR = 250;

    public static final double DEFAULT_FLYWHEEL_VELOCITY = 7000;
    public static final double SLOW_FLYWHEEL_VELOCITY = 5000;
    public static final double FAST_FLYWHEEL_VELOCITY = 10000;
    private double targetFlywheelVelocity = DEFAULT_FLYWHEEL_VELOCITY;

    public boolean spinningFlywheel = false;

    // Flywheel velocity setup
    double maxVelocity = 22500;
    double encoderCPR = 4096;
    double numberOf100msIntervalsPerMinute = 600;

    private final double feedingPower = 0.2;

    @Override
    public void periodic() {
        outputTelemetry(true);
        
        if (targetState != currentState) {
            switch (currentState) {
                case IDLE -> handleIDLE();
                case CHARGING -> handleCHARGING();
                case CHARGED -> handleCHARGED();
                case SHOOTING -> handleSHOOTING();
                default -> {
                    System.out.println("Shooter FAILURE");
                    shutdownSubsystem();
                }
            }   
        }

        if (spinningFlywheel) {
            setFlywheelVelocityToCurrentTarget();
        } else {
            setFlywheelPower(0);
        }
    }

    //region State Handling
    private void handleIDLE() {
        powerFlywheel();
        currentState = ShooterSubsystemStates.CHARGING;
    }

    private void handleCHARGING() {
        if (targetState == ShooterSubsystemStates.IDLE) {
            setIDLE();
        } 
        else if (targetState == ShooterSubsystemStates.CHARGED) {
            if (atSpeed()) {
                currentState = ShooterSubsystemStates.CHARGING;
            }
        } 
        else if (targetState == ShooterSubsystemStates.SHOOTING) {
            if (atSpeed()){
                powerShootingSystem();
                currentState = ShooterSubsystemStates.SHOOTING;
            }
        }
    }

    private void handleCHARGED() {
        if (targetState == ShooterSubsystemStates.IDLE) {
            setIDLE();
        } 
        else if (!atSpeed()) {
            currentState = ShooterSubsystemStates.CHARGING;
        } 
        else if (targetState == ShooterSubsystemStates.SHOOTING) {
            powerShootingSystem();
            currentState = ShooterSubsystemStates.SHOOTING;
        }
    }

    private void handleSHOOTING() {
        if (targetState == ShooterSubsystemStates.IDLE) {
            setIDLE();
        } 
        else if (!atSpeed()){
            currentState = ShooterSubsystemStates.CHARGING;
        } 
        else if (targetState == ShooterSubsystemStates.CHARGING ||
                 targetState == ShooterSubsystemStates.CHARGED) {
            idleFeeder();
            currentState = ShooterSubsystemStates.CHARGING;
        }
    }

    private void setIDLE() {
        idleShootingSystem();
        currentState = ShooterSubsystemStates.IDLE;
    }
    //endregion

    public Command setFlywheelVelocityCommand(double velocity) {
        return runOnce(() -> {
            setFlywheelVelocity(velocity);
        });
    }

    public Command setTargetFlywheelVelocity(double velocity) {
        return runOnce(() -> { 
            targetFlywheelVelocity = velocity; 

            if (currentState != ShooterSubsystemStates.IDLE) {
                powerFlywheel();
            }
        });
    }

    private double RPMtoTalonUnits(double rpm) {
        return rpm * encoderCPR / numberOf100msIntervalsPerMinute;
    }

    private void idleFlywheel() {
        spinningFlywheel = false;
    }

    private void powerFlywheel() {
        spinningFlywheel = true;
    }

    private void idleFeeder() {
        setFeederPower(0);
    }

    private void powerFeeder() {
        setFeederPower(feedingPower);
    }

    private void idleShootingSystem() {
        idleFlywheel();
        idleFeeder();
    }

    private void powerShootingSystem() {
        powerFeeder();
        powerFlywheel();
    }

    private boolean atSpeed() {
        return Math.abs(TargetToActual(flywheelMotor.getSelectedSensorVelocity()) - targetFlywheelVelocity) <= FLYWHEEL_CLOSED_LOOP_ERROR;
    }

    private void setFlywheelVelocityToCurrentTarget() {
        setFlywheelVelocity(targetFlywheelVelocity);
    }

    private void setFlywheelVelocity(double velocity) {
        flywheelMotor.set(ControlMode.Velocity, TargetToActual(velocity));
    }

    /*
     * Converters the actual rpm to the proper RPM after using TargetToActual()
     */
    private double ActualToTarget(double target) {
        return (0.982 * target) - 469;
    }

    /*
     * Converts the RPM to what is actually needed to help with motor speed offset
     */
    private double TargetToActual(double actual) {
        return (actual + 469) * 0.982;
    }

    private void setFlywheelPower(double power) {
        flywheelMotor.set(ControlMode.PercentOutput, power);
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
        targetState = state;
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
        if (telemetryTimer.hasElapsed(1) && enableTelemetry) {
            
            System.out.println("Current Shooter State: " + currentState);
            System.out.println("Spinning flywheel: " + spinningFlywheel);
            System.out.println(
                "Flywheel Target = " + targetFlywheelVelocity +
                "\nFlywheel Speed = " + flywheelMotor.getSelectedSensorVelocity() +
                "\n (At speed: " + atSpeed() + ")");
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

        flywheelMotor.setNeutralMode(NeutralMode.Coast);
        flywheelMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 30);

        flywheelMotor.enableVoltageCompensation(true);
        flywheelMotor.configVoltageCompSaturation(12.0);

        flywheelMotor.enableCurrentLimit(true);
        flywheelMotor.configPeakCurrentLimit(30);
        
        flywheelMotor.setSensorPhase(false);

        flywheelMotor.configAllowableClosedloopError(0, FLYWHEEL_CLOSED_LOOP_ERROR);
        
        flywheelMotor.config_kP(0, 0.1);
        flywheelMotor.config_kI(0, 0.0);
        flywheelMotor.config_kD(0, 0.0);

        double maxTalonUnits = RPMtoTalonUnits(maxVelocity);
        double kF = 1023 / maxTalonUnits;
        flywheelMotor.config_kF(0, kF);
    }
}
