package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;


/**
     * The {@link #telemetry} field contains an object in which a user may accumulate data which
     * is to be transmitted to the driver station. This data is automatically transmitted to the
     * driver station on a regular, periodic basis.
     */
public class Uppies extends BaseHardware{

    public HardwareMap hardwareMap = null; // will be set in Child class

    public Mode CurrentMode;
    public Servo USC;
    public boolean UP = false;
    public Servo USCC;

public static final double UpUSCC = 0.1;
public static final double DownUSCC = 1;
    public static final double UpUSC = 1;
    public static final double DownUSC = 0.1;
    /**
     * BaseHardware constructor
     * <p>
     * The op mode name should be unique. It will be the name displayed on the driver station. If
     * multiple op modes have the same name, only one will be available.
     */
    public Uppies() {

    }

    /**
     * User defined init method
     * <p>
     * This method will be called once when the INIT button is pressed.
     */
    public void init() {

    }

    /**
     * User defined init_loop method
     * <p>
     * This method will be called repeatedly when the INIT button is pressed.
     * This method is optional. By default this method takes no action.
     */
    public void init_loop() {

    }

    /**
     * User defined start method.
     * <p>
     * This method will be called once when the PLAY button is first pressed.
     * This method is optional. By default this method takes not action.
     * Example usage: Starting another thread.
     */
    public void start() {

    }

    /**
     * User defined loop method
     * <p>
     * This method will be called repeatedly in a loop while this op mode is running
     */
    public void loop() {

    }

    /**
     * User defined stop method
     * <p>
     * This method will be called when this op mode is first disabled
     * <p>
     * The stop method is optional. By default this method takes no action.
     */
    void stop() {

    }

    public void cmdUp(){
        CurrentMode = Mode.UP;
        USC.setPosition(UpUSC);
        USCC.setPosition(UpUSCC);
    }

    public void cmdDown(){
        CurrentMode = Mode.DOWN;
        USC.setPosition(DownUSC);
        USCC.setPosition(DownUSCC);

    }

    public enum Mode{
        UP,
        DOWN
    }
}
