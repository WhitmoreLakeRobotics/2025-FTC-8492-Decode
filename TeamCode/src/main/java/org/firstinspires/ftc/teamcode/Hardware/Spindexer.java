package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Common.CommonLogic;


/**
 * Base class for FTC Team 8492 defined hardware
 */
public class Spindexer {


    /**
     * The {@link #telemetry} field contains an object in which a user may accumulate data which
     * is to be transmitted to the driver station. This data is automatically transmitted to the
     * driver station on a regular, periodic basis.
     */
    public double SDM01Power;

    private DcMotor SDM01;
    private static final double NormalSpeed = 0.25;
    private static final double TurboSpeed = 0.75;

    public final double minPower = -1.0;
    public final double maxPower = 1.0;

    public Mode CurrentMode;



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
         SDM01.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         SDM01.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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


    }

   public void NTK_POSITION_ONE(){
       CurrentMode = Mode.SDin1;
       SDM01.setTargetPosition(0);
   }

    public void NTK_POSITION_TWO(){
        CurrentMode = Mode.SDin2;
        SDM01.setTargetPosition(120);
    }
    public void NTK_POSITION_THREE(){
        CurrentMode = Mode.SDin3;
        SDM01.setTargetPosition(-120);
    }

    public void OTK_POSITION_ONE(){
        CurrentMode = Mode.SDout1;
        SDM01.setTargetPosition(-180);
    }

    public void OTK_POSITION_TWO(){
        CurrentMode = Mode.SDout2;
        SDM01.setTargetPosition(-60);
    }

    public void OTK_POSITION_THREE(){
        CurrentMode = Mode.SDout3;
        SDM01.setTargetPosition(60);
    }

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

//*pure confusion*

}






















