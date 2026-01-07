package frc.robot.subsystems;

public interface CustomSubsystem<S extends Enum<S>> {

    public S getState();

    // Now only valid states for THIS subsystem can be passed in
    public void setTargetState(S state);

    public void shutdownSubsystem();
    public void resetSubsystem();
    public void outputTelemetry(boolean enableTelemetry);
    public void initializeSubsystem();
}
