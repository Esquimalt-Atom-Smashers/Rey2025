package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CustomSubsystem;
import frc.robot.subsystems.PhoenixIDConstants;

public class AimSubsystem extends SubsystemBase implements CustomSubsystem<AimSubsystem.AimingSubsystemStates> {

    private AimingSubsystemStates currentState = AimingSubsystemStates.IDLE;
    private AimingSubsystemStates targetState = AimingSubsystemStates.IDLE;

    private Timer telemetryTimer = new Timer();

    private double targetPosition;

    public enum AimingSubsystemStates {
        IDLE,
        AIMING,
        AIMED
    }

    private VictorSPX aimingPanelRotator = new VictorSPX(PhoenixIDConstants.HOOD_LINEAR_ACTUATOR);

    @Override
    public void periodic() {
        outputTelemetry(true);

        if (targetState != currentState) {
            switch (currentState) {
                case IDLE:
                    setAimingPanelPosition(targetPosition);

                    if (targetState == AimingSubsystemStates.AIMED || targetState == AimingSubsystemStates.AIMING) {
                        currentState = AimingSubsystemStates.AIMING;
                    }

                case AIMING:
                    if (targetState == AimingSubsystemStates.IDLE) {
                        setAimingPanelPower(0);
                        currentState = AimingSubsystemStates.IDLE;
                    }
                    else if (targetState == AimingSubsystemStates.AIMED) {
                        if (atTargetPosition()) {
                            currentState = AimingSubsystemStates.AIMED;
                        } else {
                            setAimingPanelPosition(targetPosition);
                        }
                    }
                    break;

                case AIMED:
                    if (targetState == AimingSubsystemStates.IDLE) {
                        setAimingPanelPower(0);
                        currentState = AimingSubsystemStates.IDLE;
                    } else if (!atTargetPosition()) {
                        currentState = AimingSubsystemStates.AIMING;
                    }
                    break;
                default:
                    setTargetState(AimingSubsystemStates.IDLE);
            }
        }
    }

    public void setAimingPanelPower(double power) {
        aimingPanelRotator.set(ControlMode.PercentOutput, power);
    }

    public void setAimingPanelPosition(double position) {
        aimingPanelRotator.set(ControlMode.Position, position);
    }

    public boolean atPosition(double position) {
        return aimingPanelRotator.getSelectedSensorPosition() >= targetPosition;
    }

    public boolean atTargetPosition() {
        return atPosition(targetPosition);
    }

    public void setTargetPosition(double position) {
        targetPosition = position;
    }

    @Override
    public AimingSubsystemStates getState() {
        return currentState;
    }

    @Override
    public void setTargetState(AimingSubsystemStates state) {
        targetState = state;
    }

    @Override
    public void shutdownSubsystem() {
        
    }

    @Override
    public void resetSubsystem() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'resetSubsystem'");
    }

    @Override
    public void outputTelemetry(boolean enableTelemetry) {
        if (telemetryTimer.hasElapsed(1)) {
            System.out.println("Aiming panel target position: " + targetPosition + "/" + aimingPanelRotator.getSelectedSensorPosition() + " (At pos: " + atTargetPosition() + ")");
            System.out.println("Current aiming panel state " + currentState);
            telemetryTimer.reset();
        }
    }

    @Override
    public void initializeSubsystem() {
        aimingPanelRotator.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 30);
        
        aimingPanelRotator.configFactoryDefault();

        aimingPanelRotator.config_kP(0, 0.1);
        aimingPanelRotator.config_kI(0, 0.0);
        aimingPanelRotator.config_kD(0, 0.0);

        aimingPanelRotator.setNeutralMode(NeutralMode.Brake);
        aimingPanelRotator.setInverted(false);
    }
}
