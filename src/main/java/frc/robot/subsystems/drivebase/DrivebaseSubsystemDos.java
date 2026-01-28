package frc.robot.subsystems.drivebase;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CustomSubsystem;

public class DrivebaseSubsystemDos extends SubsystemBase {
    
    private DrivebaseSubsystemDosStates currentState = DrivebaseSubsystemDosStates.IDLE;
    //left half motors
    private final VictorSPX leftDrive1 = new VictorSPX(1);
    private final TalonSRX leftTalon = new TalonSRX(2);
    private final VictorSPX leftDrive2 = new VictorSPX(3);
    
    //right half motors
    private final VictorSPX rightDrive1 = new VictorSPX(4);
    private final TalonSRX rightTalon = new TalonSRX(5);
    private final VictorSPX rightDrive2 = new VictorSPX(6);

    //talon motor encoders
    private DutyCycleEncoder leftEncoder;
    private DutyCycleEncoder rightEncoder;

    private Supplier <Double> turnSupplier;
    private Supplier <Double> driveSupplier;

    private double slowModeMultiplier;
    public enum DrivebaseSubsystemDosStates {

        IDLE,
        
        SLOWMODE,
        
        MANUAL_CONTROL
    
    }
    
    public DrivebaseSubsystemDos(Supplier <Double> turnSupplier, Supplier <Double> driveSupplier) {
       
        this.driveSupplier = driveSupplier;
        this.turnSupplier = turnSupplier;

    }   

    private void setMotorPowers(double leftPower, double rightPower, boolean slowMode) {
        
        if (slowMode){
            slowModeMultiplier = 0.1;
        } 
        
        else {
            slowModeMultiplier = 1;
        }

        leftDrive1.set(ControlMode.PercentOutput, leftPower * slowModeMultiplier);
        leftTalon.set(ControlMode.PercentOutput, leftPower * slowModeMultiplier);
        leftDrive2.set(ControlMode.PercentOutput, leftPower * slowModeMultiplier);

        rightDrive1.set(ControlMode.PercentOutput, rightPower * slowModeMultiplier);
        rightTalon.set(ControlMode.PercentOutput, rightPower * slowModeMultiplier);
        rightDrive2.set(ControlMode.PercentOutput, rightPower * slowModeMultiplier);

    }
    
    public Command setDriveStateCommand(DrivebaseSubsystemDosStates state) {

        return runOnce(() -> {setTargetState(state);});

    }

    

    //runs every 20ms
    public void periodic() {
        
        double drive = turnSupplier.get();
        double turn = driveSupplier.get();
        
        double leftPower = (drive - turn);
        double rightPower = (drive + turn);

        double max = Math.max(Math.abs(rightPower),Math.abs(leftPower));
            if (max>1){
                leftPower /= max;
                rightPower /= max;
            }

        switch (currentState) {
            
            case MANUAL_CONTROL:
                
                setMotorPowers( leftPower, rightPower, false);
                
                break;
            
            case IDLE:
                
                setMotorPowers(0, 0, false);
                
                break;
            
            case SLOWMODE:
                
                setMotorPowers(leftPower, rightPower, true);
                
                break;
            
            default:
                break;
            
        }
    }

    // you'll never guess this but it initializes the subsystem
    public void initializeSubsystem() {
        
        leftDrive1.configFactoryDefault();
        leftTalon.configFactoryDefault();
        leftDrive2.configFactoryDefault();
        rightDrive1.configFactoryDefault();
        rightTalon.configFactoryDefault();
        rightDrive2.configFactoryDefault();
        
        leftDrive1.setInverted(true);
        leftTalon.setInverted(true);
        leftDrive2.setInverted(true);

        leftEncoder = new DutyCycleEncoder(0);
        rightEncoder = new DutyCycleEncoder(1);

        leftDrive1.setNeutralMode(NeutralMode.Brake);
        leftTalon.setNeutralMode(NeutralMode.Brake);
        leftDrive2.setNeutralMode(NeutralMode.Brake);
        rightDrive1.setNeutralMode(NeutralMode.Brake);
        rightTalon.setNeutralMode(NeutralMode.Brake);
        rightDrive2.setNeutralMode(NeutralMode.Brake);

    }

    public void setTargetState(DrivebaseSubsystemDosStates state) {
        currentState = state;
    }
}

