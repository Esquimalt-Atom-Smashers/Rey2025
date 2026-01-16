package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CustomSubsystem;
import frc.robot.subsystems.PhoenixIDConstants;

public class AimSubsystem extends SubsystemBase implements CustomSubsystem<AimSubsystem.AimingSubsystemStates> {

    private AimingSubsystemStates currentState = AimingSubsystemStates.IDLE;
    private AimingSubsystemStates targetState = AimingSubsystemStates.IDLE;

    private Timer telemetryTimer = new Timer();

    public enum AimingSubsystemStates {
        IDLE,
        AIMING,
        AIMED
    }

    private VictorSPX aimingPanelRotator = new VictorSPX(PhoenixIDConstants.HOOD_LINEAR_ACTUATOR);

    @Override
    public void periodic() {
        outputTelemetry(true);
    }

    public void setAimingPanelPower(double power) {
        aimingPanelRotator.set(ControlMode.PercentOutput, power);
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
            telemetryTimer.reset();
        }
    }

    @Override
    public void initializeSubsystem() {
        
    }
}
