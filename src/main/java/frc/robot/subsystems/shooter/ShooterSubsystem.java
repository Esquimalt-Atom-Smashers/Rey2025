package frc.robot.subsystems.shooter;

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
    private final double TOLERANCE = 2400;
    // create transferSubsystem states here

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
    private final PIDController flywheelVelocityController = new PIDController(0.5, 0.0, 0.0);

    private final VictorSPX feederMotor = new VictorSPX(PhoenixIDConstants.SHOOTER_FEEDER);

    public static final double DEFAULT_FLYWHEEL_VELOCITY = 3000;
    public static final double SLOW_FLYWHEEL_VELOCITY = 2000;
    public static final double FAST_FLYWHEEL_VELOCITY = 6000;
    private double targetFlywheelVelocity = DEFAULT_FLYWHEEL_VELOCITY;

    // Flywheel velocity setup
    double maxRPM = 2000;
    double encoderCPR = 4096;
    double maxTalonUnits = maxRPM * encoderCPR / 600;

    private final double feedingPower = 0.2;

    @Override
    public void periodic() {
        outputTelemetry(false);
        
        if (targetState != currentState) {
            switch (currentState) {
                case IDLE: //for all other states we want to start the flywheel and move to the charging state

                    powerFlywheel();
                    currentState = ShooterSubsystemStates.CHARGING;
                    break;

                case CHARGING:

                    if (targetState == ShooterSubsystemStates.IDLE) {
                        idleShootingSystem();
                        setCurrentState(ShooterSubsystemStates.IDLE);
                    } 
                    else if (targetState == ShooterSubsystemStates.CHARGED) {
                        
                        if (atSpeed()) {
                            setCurrentState(ShooterSubsystemStates.CHARGED);
                        }
                    } 
                    else if (targetState == ShooterSubsystemStates.SHOOTING) {
                        
                        if (atSpeed()){
                            powerShootingSystem();
                            setCurrentState(ShooterSubsystemStates.SHOOTING);
                        }
                    }
                    break;

                case CHARGED:

                    if (targetState == ShooterSubsystemStates.IDLE) {
                        idleShootingSystem();
                        setCurrentState(ShooterSubsystemStates.IDLE);
                    } 
                    else if (!atSpeed()) {
                        setCurrentState(ShooterSubsystemStates.CHARGING);
                    } 
                    else if (targetState == ShooterSubsystemStates.SHOOTING) {
                        powerShootingSystem();
                        setCurrentState(ShooterSubsystemStates.SHOOTING);
                    }
                    break;

                case SHOOTING:
                    
                    if (targetState == ShooterSubsystemStates.IDLE) {
                        idleShootingSystem();
                        setCurrentState(ShooterSubsystemStates.IDLE);
                    } 
                    else if (!atSpeed()){
                        setCurrentState(ShooterSubsystemStates.CHARGING);
                    } 
                    else if (targetState == ShooterSubsystemStates.CHARGING ||
                             targetState == ShooterSubsystemStates.CHARGED) {
                        idleFeeder();
                        setCurrentState(ShooterSubsystemStates.CHARGING);
                    }
                    break;
                default:
                    System.out.println("Shooter FAILURE");
                    shutdownSubsystem();
                    break;
            }   
        }
    }

    public Command setFlywheelVelocityCommand(double velocity) {
        return runOnce(() -> {
            setFlywheelVelocity(velocity);
        });
    }

    public Command setTargetFlywheelVelocity(double velocity) {
        return runOnce(() -> { 
            targetFlywheelVelocity = velocity; 

            if (currentState != ShooterSubsystemStates.IDLE) {
                setFlywheelVelocityToCurrentTarget();
            }
        });
    }
    
    private void idleFlywheel() {
        setFlywheelPower(0);
    }

    private void powerFlywheel() {
        setFlywheelVelocityToCurrentTarget();
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
        return Math.abs(flywheelMotor.getClosedLoopError()) >= targetFlywheelVelocity * 4;
    }

    private void setFlywheelVelocityToCurrentTarget() {
        setFlywheelVelocity(targetFlywheelVelocity);
    }

    private void setFlywheelVelocity(double velocity) {
        flywheelMotor.set(ControlMode.Velocity, getFlywheelRPM(velocity));
    }

    private void setFlywheelPower(double power) {
        flywheelMotor.set(ControlMode.PercentOutput, power);
    }

    private double getFlywheelRPM(double velocity) {
        return velocity * encoderCPR / 600;
    }

    private void setFeederPower(double power) {
        feederMotor.set(ControlMode.PercentOutput, power);
    }

    private void setCurrentState(ShooterSubsystemStates state) {
        currentState = state;
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
            
            System.out.println("Flywheel RPM = " + Math.abs(flywheelMotor.getClosedLoopError()) + "/" + targetFlywheelVelocity * 4);
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

        flywheelMotor.enableCurrentLimit(true);
        flywheelMotor.configPeakCurrentLimit(2);
        flywheelMotor.config_kP(0, 0.1);
        flywheelMotor.config_kI(0, 0.0);
        flywheelMotor.config_kD(0, 0.0);

        double kF = 1023 / maxTalonUnits;
        flywheelMotor.config_kF(0, kF);
    }
}
