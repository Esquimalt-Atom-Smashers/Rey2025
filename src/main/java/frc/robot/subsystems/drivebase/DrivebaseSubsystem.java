package frc.robot.subsystems.drivebase;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CustomSubsystem;
import frc.robot.subsystems.PhoenixIDConstants;
public class DrivebaseSubsystem extends SubsystemBase implements CustomSubsystem<DrivebaseSubsystem.DrivebaseSubsystemStates> {
    // create transferSubsystem states here
    private DrivebaseSubsystemStates currentState = DrivebaseSubsystemStates.IDLE;
    // left half motors
    private final VictorSPX L1 = new VictorSPX(PhoenixIDConstants.LEFT_DRIVE_VICTOR_1);
    private final TalonSRX L2 = new TalonSRX(PhoenixIDConstants.LEFT_DRIVE_TALON);
    private final VictorSPX L3 = new VictorSPX(PhoenixIDConstants.LEFT_DRIVE_VICTOR_2);
    // right half motors
    private final VictorSPX R1 = new VictorSPX(PhoenixIDConstants.RIGHT_DRIVE_VICTOR_1);
    private final TalonSRX R2 = new TalonSRX(PhoenixIDConstants.RIGHT_DRIVE_TALON);
    private final VictorSPX R3 = new VictorSPX(PhoenixIDConstants.RIGHT_DRIVE_VICTOR_2);

    private  DutyCycleEncoder L2Encoder;
    private  DutyCycleEncoder R2Encoder;

    private double powerLimiter = 0.1;

    public enum DrivebaseSubsystemStates {
        IDLE,
        MANUAL_CONTROL,
        thirdstate
    }
    public DrivebaseSubsystem() {
        // makes it so it starts from a known position
        L1.configFactoryDefault(); 
        L2.configFactoryDefault();
        L3.configFactoryDefault();
        R1.configFactoryDefault();
        R2.configFactoryDefault();
        R3.configFactoryDefault();

        //inverts motors to match the left side
        R1.setInverted(true);
        R2.setInverted(true);
        R3.setInverted(true);

        L2Encoder = new DutyCycleEncoder(0);
        R2Encoder = new DutyCycleEncoder(1);
    }

    public Command drive(Supplier <Double> driveSupplier, Supplier <Double> turnSupplier) {

        return run(()-> {
            
            currentState = DrivebaseSubsystemStates.MANUAL_CONTROL;
            double drive = driveSupplier.get();
            double turn = turnSupplier.get();
            double leftPower = (drive + turn);
            double rightPower = (drive - turn);
            double max = Math.max(Math.abs(rightPower),Math.abs(leftPower));
            if (max>1){
                leftPower /= max;
                rightPower /= max;
            }
            setMotorPowers(leftPower, rightPower);
        });
    }
    private void setMotorPowers(double leftPower, double rightPower){
        
        L1.set(ControlMode.PercentOutput, leftPower * powerLimiter);
        L2.set(ControlMode.PercentOutput, leftPower * powerLimiter);
        L3.set(ControlMode.PercentOutput, leftPower * powerLimiter);
        R1.set(ControlMode.PercentOutput, rightPower * powerLimiter);
        R2.set(ControlMode.PercentOutput, rightPower * powerLimiter);
        R3.set(ControlMode.PercentOutput, rightPower * powerLimiter);
    }




    @Override
    public void periodic() {
        // This runs every 20ms. Use it to act on the current state.
        switch (currentState) {
            case MANUAL_CONTROL:

                break;
            case IDLE:
                L1.set(ControlMode.PercentOutput, 0);
                L2.set(ControlMode.PercentOutput, 0);
                L3.set(ControlMode.PercentOutput, 0);
                R1.set(ControlMode.PercentOutput, 0);
                R2.set(ControlMode.PercentOutput, 0);
                R3.set(ControlMode.PercentOutput, 0);
                // Stop motors
                break;
            default:
                break;
        }
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
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'initializeSubsystem'");
    }
    

}
