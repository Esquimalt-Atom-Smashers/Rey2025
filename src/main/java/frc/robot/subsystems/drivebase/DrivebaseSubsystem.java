package frc.robot.subsystems.drivebase;


import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CustomSubsystem;
import frc.robot.subsystems.PhoenixIDConstants;
public class DrivebaseSubsystem extends SubsystemBase implements CustomSubsystem<DrivebaseSubsystem.DrivebaseSubsystemStates> {
    // create transferSubsystem states here
    private DrivebaseSubsystemStates currentState = DrivebaseSubsystemStates.IDLE;
    private DrivebaseSubsystemStates targetState = DrivebaseSubsystemStates.IDLE;

    private Timer telemetryTimer = new Timer();

    private final VictorSPX leftDrive1 = new VictorSPX(PhoenixIDConstants.LEFT_DRIVE_VICTOR_1);
    private final TalonSRX leftDriveTalon = new TalonSRX(PhoenixIDConstants.LEFT_DRIVE_TALON);
    private final VictorSPX leftDrive2 = new VictorSPX(PhoenixIDConstants.LEFT_DRIVE_VICTOR_2);
    
    private final VictorSPX rightDrive1 = new VictorSPX(PhoenixIDConstants.RIGHT_DRIVE_VICTOR_1);
    private final TalonSRX rightDriveTalon = new TalonSRX(PhoenixIDConstants.RIGHT_DRIVE_TALON);
    private final VictorSPX rightDrive2 = new VictorSPX(PhoenixIDConstants.RIGHT_DRIVE_VICTOR_2);

    private final Supplier<Double> driveSupplier;
    private final Supplier<Double> turnSupplier;

    public final double DEFAULT_SPEED = 0.3;
    public final double SLOW_SPEED = 0.1;

    private boolean useSlowMode = false;

    public enum DrivebaseSubsystemStates {
        IDLE,
        MANUAL_DRIVE,
        AUTO_DRIVE
    }

    public DrivebaseSubsystem(Supplier<Double> driveSupplier, Supplier<Double> turnSupplier) {
        this.driveSupplier = driveSupplier;
        this.turnSupplier = turnSupplier;
    }

    @Override
    public void periodic() {
        outputTelemetry(true);

        drive(driveSupplier, turnSupplier, getSpeedMultiplier(useSlowMode));
    }

    public void drive(Supplier<Double> driveSupplier, Supplier<Double> turnSupplier, double speedMutliplier) {
        double drive = driveSupplier.get();
        double turn = turnSupplier.get();

        double max = Math.max(Math.abs(drive), Math.abs(turn));

        if (max > 1) {
            drive /= max;
            turn /= max;
        }

        drive(drive, turn, speedMutliplier);
    }

    private void drive(double drive, double turn, double powerMultiplier) {
        double leftPower = (drive + turn) * powerMultiplier;
        double rightPower = (drive - turn) * powerMultiplier;

        setMotorPowers(leftPower, rightPower);
    }

    private void setMotorPowers(double leftPower, double rightPower) {
        leftDrive1     .set(ControlMode.PercentOutput, leftPower);
        leftDriveTalon .set(ControlMode.PercentOutput, leftPower);
        leftDrive2     .set(ControlMode.PercentOutput, leftPower);

        rightDrive1    .set(ControlMode.PercentOutput, rightPower);
        rightDriveTalon.set(ControlMode.PercentOutput, rightPower);
        rightDrive2    .set(ControlMode.PercentOutput, rightPower);
    }

    private double getSpeedMultiplier(boolean slowMode) {
        return slowMode ? SLOW_SPEED : DEFAULT_SPEED;
    }
    
    private void idleDrive() {
        setMotorPowers(0, 0);
    }

    public void setSlowSpeed() {
        useSlowMode = true;
    }

    public void setDefaultSpeed() {
        useSlowMode = false;
    }

    @Override
    public DrivebaseSubsystemStates getState() {
        return currentState;
    }

    @Override
    public void setTargetState(DrivebaseSubsystemStates state) {
        targetState = state;
    }

    @Override
    public void shutdownSubsystem() {
        idleDrive();
    }

    @Override
    public void resetSubsystem() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'resetSubsystem'");
    }

    @Override
    public void outputTelemetry(boolean enableTelemetry) {
        if (enableTelemetry)
            return;

        if (telemetryTimer.hasElapsed(1)) {
            telemetryTimer.reset();

            System.out.println("Current speed multiplier: " + getSpeedMultiplier(useSlowMode));
        }
    }

    @Override
    public void initializeSubsystem() {
        // Left drive setup
        leftDrive1.configFactoryDefault();
        leftDriveTalon.configFactoryDefault();
        leftDrive2.configFactoryDefault();

        leftDrive1.setNeutralMode(NeutralMode.Brake);
        leftDriveTalon.setNeutralMode(NeutralMode.Brake);
        leftDrive2.setNeutralMode(NeutralMode.Brake);

        leftDrive1.setInverted(true);
        leftDriveTalon.setInverted(true);
        leftDrive2.setInverted(true);

        // Right drive setup
        rightDrive1.configFactoryDefault();
        rightDriveTalon.configFactoryDefault();
        rightDrive2.configFactoryDefault();

        rightDrive1.setNeutralMode(NeutralMode.Brake);
        rightDriveTalon.setNeutralMode(NeutralMode.Brake);
        rightDrive2.setNeutralMode(NeutralMode.Brake);
    }
}
