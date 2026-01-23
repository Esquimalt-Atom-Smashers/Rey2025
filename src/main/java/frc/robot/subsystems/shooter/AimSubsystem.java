package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CustomSubsystem;
import frc.robot.subsystems.PhoenixIDConstants;

public class AimSubsystem extends SubsystemBase implements CustomSubsystem<AimSubsystem.AimingSubsystemStates> {

    private AimingSubsystemStates currentState = AimingSubsystemStates.IDLE;
    private AimingSubsystemStates targetState = AimingSubsystemStates.IDLE;

    private boolean aiming;

    private final Timer telemetryTimer = new Timer();

    private double targetPosition = hoodDownPosition;

    public enum AimingSubsystemStates {
        IDLE,
        AIMING,
        AIMED
    }

    private final VictorSPX hoodMotor = new VictorSPX(PhoenixIDConstants.HOOD_LINEAR_ACTUATOR);
    private final AnalogInput hoodPotentiometer = new AnalogInput(0);

    PIDController pid = new PIDController(2.0, 0.0, 0.1);

    // Customization

    // 0 - 5 "volts"
    public static final double hoodDownPosition = 4.9;
    public static final double hoodHalfwayPosition = 3.5;
    public static final double hoodUpPosition = 2.85;

    private final double maxHoodSpeed = 0.5;

    @Override
    public void periodic() {
        outputTelemetry(false);

        if (targetState != currentState) {
            switch (currentState) {
                case IDLE -> handleIDLE();
                case AIMING -> handleAIMING();
                case AIMED -> handleAIMED();
                default -> setTargetState(AimingSubsystemStates.IDLE);
            }
        }

        if (aiming) {
            setAimingPanelPosition(targetPosition);
        } else {
            setAimingPanelPower(0);
        }
    }

    //region State Handling
    private void handleIDLE() {
        aiming = true;
        currentState = AimingSubsystemStates.AIMING;
    }

    private void handleAIMING() {
        if (targetState == AimingSubsystemStates.IDLE) {
            setIDLE();
        }
        else if (targetState == AimingSubsystemStates.AIMED) {
            if (atTargetPosition()) {
                currentState = AimingSubsystemStates.AIMED;
            } else {
                aiming = true;
            }
        }
    }

    private void handleAIMED() {
        if (targetState == AimingSubsystemStates.IDLE) {
            setIDLE();
        } else if (!atTargetPosition()) {
            currentState = AimingSubsystemStates.AIMING;
        }
    }

    private void setIDLE() {
        setAimingPanelPower(0);
        aiming = false;
        currentState = AimingSubsystemStates.IDLE;
    }
    //endregion

    public void setAimingPanelPower(double power) {
        hoodMotor.set(ControlMode.PercentOutput, power);
    }

    public Command setAimingPanelPositionCommand(double position) {
        return run (() -> { setAimingPanelPosition(position); });
    }

    private void setAimingPanelPosition(double position) {
        double output = pid.calculate(hoodPotentiometer.getVoltage(), position);
        output = MathUtil.clamp(output, -maxHoodSpeed, maxHoodSpeed);

        hoodMotor.set(ControlMode.PercentOutput, output);
    }

    private boolean atPosition(double position) {
        double tolerance = 0.02;
        return Math.abs(hoodPotentiometer.getVoltage() - position) < tolerance;
    }

    private boolean atTargetPosition() {
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
        setAimingPanelPower(0);
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
        pid.setTolerance(0.03);

        hoodMotor.configFactoryDefault();

        hoodMotor.setNeutralMode(NeutralMode.Brake);
        hoodMotor.setInverted(true);

        telemetryTimer.start();
    }
}
