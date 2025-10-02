package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;





/**
 * Base class for FTC Team 8492 defined hardware
 */
public class Intake extends BaseHardware{


    /**
     * The {@link #telemetry} field contains an object in which a user may accumulate data which
     * is to be transmitted to the driver station. This data is automatically transmitted to the
     * driver station on a regular, periodic basis.
     */
    public Telemetry telemetry = null;

    /**
     * Hardware Mappings
     */
    public HardwareMap hardwareMap = null; // will be set in Child class


    /**
     * BaseHardware constructor
     * <p>
     * The op mode name should be unique. It will be the name displayed on the driver station. If
     * multiple op modes have the same name, only one will be available.
     */
    private DcMotor NTKM01;

    public Mode CurrentMode;

    private double NTKM01Power;

    public final double minPower = -1.0;
    public final double maxPower = 1.0;

    public static final double stopSpeed = 0;
    public static final double inSpeed = 0.5;
    public static final double outSpeed = -0.5;
  //  public static final double snailoutSpeed = -0.25;


    /**
     * User defined init method
     * <p>
     * This method will be called once when the INIT button is pressed.
     */
    public void init() {



        NTKM01 = hardwareMap.get(DcMotor.class, "NTKM01");

    }

    /**
     * User defined init_loop method
     * <p>
     * This method will be called repeatedly when the INIT button is pressed.
     * This method is optional. By default this method takes no action.
     */
    public void init_loop(){

    }

    /**
     * User defined start method.
     * <p>
     * This method will be called once when the PLAY button is first pressed.
     * This method is optional. By default this method takes not action.
     * Example usage: Starting another thread.
     */
    public void start(){

    }

    /**
     * User defined loop method
     * <p>
     * This method will be called repeatedly in a loop while this op mode is running
     */
    public void loop(){

    }

    /**
     * User defined stop method
     * <p>
     * This method will be called when this op mode is first disabled
     * <p>
     * The stop method is optional. By default this method takes no action.
     */
    void stop(){

    }

    public void cmdBackward(){
        CurrentMode = Mode.NTKbackward;
        NTKM01.setPower (outSpeed);
    }
    public void cmdFoward(){
        CurrentMode = Mode.NTKforward;
        NTKM01.setPower (inSpeed);

    }

    public void cmdStop(){
        CurrentMode = Mode.NTKstop;
        NTKM01.setPower (stopSpeed);

    }

    public enum Mode {
        NTKstop,
        NTKforward,
        NTKbackward;
    }








}
