package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CustomSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
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

    private VictorSPX hoodMotor = new VictorSPX(PhoenixIDConstants.HOOD_LINEAR_ACTUATOR);
    private AnalogInput hoodPotentiometer = new AnalogInput(0);

    PIDController pid = new PIDController(2.0, 0.0, 0.1);

    private final double minHoodPosition = 3;
    private final double maxHoodPosition = 4.9;

    @Override
    public void periodic() {
        outputTelemetry(true);
    }

    public void setAimingPanelPower(double power) {
        hoodMotor.set(ControlMode.PercentOutput, power);
    }

    public Command setAimingPanelPositionCommand(double position) {
        return run (() -> { setAimingPanelPosition(position); });
    }

    public void setAimingPanelPosition(double position) {
        double maxOutput = 0.5;
        
        double output = pid.calculate(hoodPotentiometer.getVoltage(), position);
        output = MathUtil.clamp(output, -maxOutput, maxOutput);
        System.out.println(output);
        hoodMotor.set(ControlMode.PercentOutput, output);
    }

    public boolean atPosition(double position) {
        double tolerance = 0.02;
        return Math.abs(hoodPotentiometer.getVoltage() - position) < tolerance;
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
        if (telemetryTimer.hasElapsed(1) && enableTelemetry) {
            System.out.println("Aiming panel target position: " + hoodPotentiometer.getVoltage() + "/" + targetPosition + " (At pos: " + atTargetPosition() + ")");
            System.out.println("Current aiming panel state " + currentState);
            telemetryTimer.reset();
        }
    }

    @Override
    public void initializeSubsystem() {
        pid.setTolerance(0.02);

        hoodMotor.configFactoryDefault();

        hoodMotor.setNeutralMode(NeutralMode.Brake);
        hoodMotor.setInverted(true);

        telemetryTimer.start();
    }
}
