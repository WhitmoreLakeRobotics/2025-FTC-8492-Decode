package org.firstinspires.ftc.teamcode.Hardware;

import android.icu.text.Transliterator;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;


/**
 * Base class for FTC Team 8492 defined hardware
 */
@Disabled
public class Flickiteer extends BaseHardware {


    /**
     * The {@link #telemetry} field contains an object in which a user may accumulate data which
     * is to be transmitted to the driver station. This data is automatically transmitted to the
     * driver station on a regular, periodic basis.
     */


    public Servo Flicker;
    private static final double Ready = 0.0;      //change value to correct value.
    private static final double Fire = 0.45;     //change value to correct value.
    public Mode CurrentMode = Mode.Stop;



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
    public Flickiteer() {

    }

    /**
     * User defined init method
     * <p>
     * This method will be called once when the INIT button is pressed.
     */
     public void init(){

    /**
     * User defined init_loop method
     * <p>
     * This method will be called repeatedly when the INIT button is pressed.
     * This method is optional. By default this method takes no action.
     */

         Flicker = hardwareMap.get(Servo.class, "Flicker");


    }
     public void init_loop(){

    /**
     * User defined start method.
     * <p>
     * This method will be called once when the PLAY button is first pressed.
     * This method is optional. By default this method takes not action.
     * Example usage: Starting another thread.
     */
    }
     public void start(){

    /**
     * User defined loop method
     * <p>
     * This method will be called repeatedly in a loop while this op mode is running
     */
    }
     public void loop(){

    /**
     * User defined stop method
     * <p>
     * This method will be called when this op mode is first disabled
     * <p>
     * The stop method is optional. By default this method takes no action.
     */
    }
      void stop(){

    }

     public void cmdFire(){
        Flicker.setPosition(Fire);

     }

     public void cmdReady(){
         Flicker.setPosition(Ready);
     }



    public enum Mode{
        Stop
    }

}

