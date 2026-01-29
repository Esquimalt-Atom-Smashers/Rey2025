package frc.robot.subsystems.shooter;

import java.time.zone.ZoneOffsetTransitionRule.TimeDefinition;

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
    // create transferSubsystem states here
    private ShooterSubsystemStates currentState = ShooterSubsystemStates.IDLE;
    private ShooterSubsystemStates targetState = ShooterSubsystemStates.IDLE;

    private final TalonSRX shooterMotor = new TalonSRX(PhoenixIDConstants.FLYWHEEL);

    //private final PIDController flywheelVelocityController = new PIDController(0.5, 0, 0);

    private final VictorSPX feederMotor = new VictorSPX(PhoenixIDConstants.SHOOTER_FEEDER);

    public static final double DEFAULT_FLYWHEEL_VELOCITY = 3000;
    public static final double SLOW_FLYWHEEL_VELOCITY = 2000;
    public static final double FAST_FLYWHEEL_VELOCITY = 6000;
    private double targetFlywheelVelocity = DEFAULT_FLYWHEEL_VELOCITY;
    private final int FLYWHEEL_CLOSED_LOOP_ERROR = 250;
    private static final double VELOCITY_TOLERANCE = 150;

    private boolean canShoot = atSpeed();

    public enum ShooterSubsystemStates {
        CHARGING,
        CHARGED,
        SHOOTING,
        IDLE
    }

    private double maxRPM = 2000;
    private double targetRPM = 500;
    
    // ticks/counts per revolution
    private double encoderCPR = 4096;

    private double maxTalonUnits = maxRPM * encoderCPR / 600;

    private Timer telemetryTimer = new Timer();

    private final double feedingPower = 0.2;

    @Override
    public void periodic() {
        targetFlywheelVelocity = -3000;
        // This runs every 20ms. Use it to act on the current state.
        outputTelemetry(true);
        switch (currentState) {
            
            case CHARGING:
                
                setShooterVelocity(rpmToTalonUnits(targetFlywheelVelocity));
            
                if (targetState == ShooterSubsystemStates.IDLE) {
                    setCurrentState(ShooterSubsystemStates.IDLE);
                } 
            
                if (atSpeed()){
                    setCurrentState(ShooterSubsystemStates.CHARGED);
                }
                break;
            
            case CHARGED:
                
                setShooterVelocity(rpmToTalonUnits(targetFlywheelVelocity));

                if (targetState == ShooterSubsystemStates.SHOOTING) {
                    setCurrentState(ShooterSubsystemStates.SHOOTING);;
                }
                if (targetState == ShooterSubsystemStates.IDLE){
                    setCurrentState(ShooterSubsystemStates.IDLE);
                }

                break;
            
            case SHOOTING:
                
                setShooterVelocity(rpmToTalonUnits(targetFlywheelVelocity));

                if (atSpeed()) {
                    setFeederPower(-0.5);
                } else {
                    System.out.println("Shooter not ready");
                }

                if (targetState == ShooterSubsystemStates.IDLE){
                    setCurrentState(ShooterSubsystemStates.IDLE);
                }
                break;
            
            case IDLE:
                switch (targetState) {
                
                case IDLE:
                    setFeederPower(0);
                    setShooterPower(0);
                    break;

                case CHARGING:
                    setCurrentState(ShooterSubsystemStates.CHARGING);
                    break;
                
                default:
                    break;
            }
                break;
            
            default:
                break;
        }
    }
    
    private double rpmToTalonUnits(double rpm) {
        return rpm * encoderCPR / 600.0;
    }
    private double talonUnitsToRPM(double talonUnits) {
        return talonUnits * 600 / encoderCPR;
    }
    public Command setTargetStateCommand(ShooterSubsystemStates state) {
        return runOnce(() -> {setTargetState(state);});
    }

    public ShooterSubsystemStates getShooterSubsystemStates(){
        return currentState;
    }
    
    private boolean atSpeed() {
        double currentRPM = talonUnitsToRPM(shooterMotor.getSelectedSensorVelocity());
        return Math.abs(currentRPM - targetFlywheelVelocity) <= FLYWHEEL_CLOSED_LOOP_ERROR;
    }

    private void setShooterVelocity(double velocity){
        shooterMotor.set(ControlMode.Velocity, velocity);
    }

    private void setShooterPower(double power) {
        shooterMotor.set(ControlMode.PercentOutput, power);
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

    public void setCurrentState(ShooterSubsystemStates state){
        currentState = state;
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
        
        telemetryTimer.start();

        if (telemetryTimer.hasElapsed(1)) {
            System.out.println("Shooter Motor Current State is: " + currentState);
            System.out.println("Shooter Motor Target State is: " + targetState);
            System.out.println("Is it at speed?" + atSpeed());
            System.out.println("Shooter Current Speed: " + Math.abs(talonUnitsToRPM(shooterMotor.getSelectedSensorVelocity())));
            System.out.println("Shooter Target Speed" + -targetFlywheelVelocity);
            telemetryTimer.reset();
        }
        
    }

    @Override
    public void initializeSubsystem() {
        
        shooterMotor.configFactoryDefault();
        feederMotor.configFactoryDefault();

        shooterMotor.setSensorPhase(true);
        shooterMotor.setInverted(true);
        feederMotor.setInverted(false);

        shooterMotor.setNeutralMode(NeutralMode.Coast);
        shooterMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 30);
        
        shooterMotor.enableCurrentLimit(true);
        shooterMotor.configPeakCurrentLimit(30);

        shooterMotor.config_kP(0, 0.1);
        shooterMotor.config_kI(0, 0.0);
        shooterMotor.config_kD(0, 0.0);

        //double kf = 1023 / maxTalonUnits;
        //shooterMotor.config_kF(0, kf);

    }

}
