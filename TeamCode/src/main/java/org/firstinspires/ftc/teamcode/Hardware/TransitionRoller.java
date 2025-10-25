package org.firstinspires.ftc.teamcode.Hardware;

import android.transition.Transition;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;


/**
 * Base class for FTC Team 8492 defined hardware
 */
public class TransitionRoller extends BaseHardware{


    /**
     * The {@link #telemetry} field contains an object in which a user may accumulate data which
     * is to be transmitted to the driver station. This data is automatically transmitted to the
     * driver station on a regular, periodic basis.
     */

    private Mode CurrentMode;

    private DcMotor TRM01;
    private double TRPower;
    public Telemetry telemetry = null;

    public final double minPower = -1.0;
    public final double maxPower = 1.0;

    public static final double TRSpeed = 0.5;
    public static final double stopSpeed = 0.0;
     static final double TRBack = -0.5;



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
    public TransitionRoller() {

    }

    /**
     * User defined init method
     * <p>
     * This method will be called once when the INIT button is pressed.
     */
     public void init(){
        TRM01 = hardwareMap.get(DcMotor.class,"TRM01");
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
    void stop (){

    }
    /**
     * User defined stop method
     * <p>
     * This method will be called when this op mode is first disabled
     * <p>
     * The stop method is optional. By default this method takes no action.
     */
     public void cmdStop(){
         CurrentMode = Mode.Stop;
         TRM01.setPower (stopSpeed);
     }

     public void cmdSpin() {
         CurrentMode = Mode.Spin;
         TRM01.setPower(TRSpeed);
     }

     public void cmdBack() {
         CurrentMode = Mode.Back;
         TRM01.setPower(TRBack);
     }

    public enum Mode {
         Spin,
         Back,
         Stop;
    }
}
