package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;


/**
 * Base class for FTC Team 8492 defined hardware
 */
public class Spindexer {


    /**
     * The {@link #telemetry} field contains an object in which a user may accumulate data which
     * is to be transmitted to the driver station. This data is automatically transmitted to the
     * driver station on a regular, periodic basis.
     */


    private DcMotor SDM01;
    private static final double NormalSpeed = 0.25;
    private static final double TurboSpeed = 0.75;





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
    public Spindexer() {

    }

    /**
     * User defined init method
     * <p>
     * This method will be called once when the INIT button is pressed.
     */
     public void init(){

         SDM01 = hardwareMap.get (DcMotor.class, "SDM01");

     };

    /**
     * User defined init_loop method
     * <p>
     * This method will be called repeatedly when the INIT button is pressed.
     * This method is optional. By default this method takes no action.
     */
     public void init_loop(){

     };

    /**
     * User defined start method.
     * <p>
     * This method will be called once when the PLAY button is first pressed.
     * This method is optional. By default this method takes not action.
     * Example usage: Starting another thread.
     */
     public void start(){

     };

    /**
     * User defined loop method
     * <p>
     * This method will be called repeatedly in a loop while this op mode is running
     */
     public void loop(){

     };

    /**
     * User defined stop method
     * <p>
     * This method will be called when this op mode is first disabled
     * <p>
     * The stop method is optional. By default this method takes no action.
     */
   public void stop(){

    };

    public enum Mode{
        SDin1,
        SDin2,
        SDin3,
        SDout1,
        SDout2,
        SDout3;

    }


    public enum Method{
    START( ),
    NTK_POSITION_ONE(),
    NTK_POSITION_TWO(),
    NTK_POSITION_THREE(),
    OTK_POSITION_ONE(),
    OTK_POSITION_TWO(),
    OTK_POSITION_THREE();
    }



}






















