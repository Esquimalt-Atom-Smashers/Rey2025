package frc.robot.subsystems.drivebase;


import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CustomSubsystem;
import frc.robot.subsystems.PhoenixIDConstants;

public class DrivebaseSubsystem extends SubsystemBase implements CustomSubsystem<DrivebaseSubsystem.DrivebaseSubsystemStates> {
    // create transferSubsystem states here
    private DrivebaseSubsystemStates currentState = DrivebaseSubsystemStates.IDLE;
    private DrivebaseSubsystemStates targetState = DrivebaseSubsystemStates.IDLE;

    private final VictorSPX L1 = new VictorSPX(PhoenixIDConstants.LEFT_DRIVE_VICTOR_1);
    private final TalonSRX L2 = new TalonSRX(PhoenixIDConstants.LEFT_DRIVE_TALON);
    private final VictorSPX L3 = new VictorSPX(PhoenixIDConstants.LEFT_DRIVE_VICTOR_2);
    // right half motors
    private final VictorSPX R1 = new VictorSPX(PhoenixIDConstants.RIGHT_DRIVE_VICTOR_1);
    private final TalonSRX R2 = new TalonSRX(PhoenixIDConstants.RIGHT_DRIVE_TALON);
    private final VictorSPX R3 = new VictorSPX(PhoenixIDConstants.RIGHT_DRIVE_VICTOR_2);

    private double speedMultiplier = 1;

    public enum DrivebaseSubsystemStates {
        IDLE,
        secondState,
        thirdstate
    }

    @Override
    public void periodic() {
        // This runs every 20ms. Use it to act on the current state.
        switch (currentState) {
            case secondState:
                // Logic to move motors
                break;
            case IDLE:
                // Stop motors
                break;
            default:
                break;
        }
    }

    public Command drive(Supplier<Double> driveSupplier, Supplier<Double> turnSupplier) {
        return run(() -> {
            double drive = driveSupplier.get();
            double turn = driveSupplier.get();

            double max = Math.max(Math.abs(drive),Math.abs(turn));

            if (max > 1){
                drive /= max;
                turn /= max;
            }

            drive(drive, turn);
        });
    }

    private void drive(double drive, double turn) {
        double leftPower = (drive + turn) * speedMultiplier;
        double rightPower = (drive - turn) * speedMultiplier;

        L1.set(ControlMode.PercentOutput, leftPower);
        L2.set(ControlMode.PercentOutput, leftPower);
        L3.set(ControlMode.PercentOutput, leftPower);

        R1.set(ControlMode.PercentOutput, rightPower);
        R2.set(ControlMode.PercentOutput, rightPower);
        R3.set(ControlMode.PercentOutput, rightPower);
    }

    @Override
    public DrivebaseSubsystemStates getState() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getState'");
    }

    @Override
    public void setTargetState(DrivebaseSubsystemStates state) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setTargetState'");
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
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'outputTelemetry'");
    }

    @Override
    public void initializeSubsystem() {
        L1.configFactoryDefault();
        L2.configFactoryDefault();
        L3.configFactoryDefault();

        R1.configFactoryDefault();
        R2.configFactoryDefault();
        R3.configFactoryDefault();

        R1.setInverted(true);
        R2.setInverted(true);
        R3.setInverted(true);
    }
    

}
