package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

/**
 * Base class for FTC Team 8492 defined hardware
 */
public class Lighting extends BaseHardware {

    private ElapsedTime runtime = new ElapsedTime();
    /**
     * The {@link #telemetry} field contains an object in which a user may accumulate data which
     * is to be transmitted to the driver station. This data is automatically transmitted to the
     * driver station on a regular, periodic basis.
     */
    public Telemetry telemetry = null;

    private Servo PeaLight;

    public int DefaultColor;
    public int TempColor ;
    public static int TempColorTimeout = 500;
    public Color CurrentColor;

    public static final double Green = 0.5;
    public static final double Red = 0.28;
    public static final double Yellow = 0.388;
    public static final double Purple = 0.722;
    public static final double Blue = 0.6111;
    public static final double Orange = 0.333;
    public static final double Off = 0;

    public boolean initLight1 = false;
    public boolean initLight2 = false;

    private final static int GAMEPAD_LOCKOUT = 500;

    private ElapsedTime initLightTime = new ElapsedTime();

   private RevBlinkinLedDriver blinkinLedDriver;
   private RevBlinkinLedDriver.BlinkinPattern pattern = RevBlinkinLedDriver.BlinkinPattern.BLACK;
   private RevBlinkinLedDriver.BlinkinPattern baseColor = RevBlinkinLedDriver.BlinkinPattern.BLACK;
   private Telemetry.Item patternName;
   private Telemetry.Item display;
   //private RevBlinkinLedDriver.BlinkinPattern displayKind;
   private Deadline ledCycleDeadline;
   private Deadline gamepadRateLimit;

    private enum DisplayKind {
        MANUAL,
        AUTO
    }

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
    /*public Swing_Arm_And_Lift() {

    }*/

    /**
     * User defined init method
     * <p>
     * This method will be called once when the INIT button is pressed.
     */
    public void init(){
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "LEDC");
        pattern = RevBlinkinLedDriver.BlinkinPattern.WHITE;
        blinkinLedDriver.setPattern(pattern);

        PeaLight = hardwareMap.get(Servo.class,"PeaLight");

        //display = telemetry.addData("Display Kind: ", displayKind.toString());
        patternName = telemetry.addData("Pattern: ", pattern.toString());

        initLightTime.reset();
        initLight1 = true;

    }

    /**
     * User defined init_loop method
     * <p>
     * This method will be called repeatedly when the INIT button is pressed.
     * This method is optional. By default this method takes no action.
     */
     public void init_loop(){

         if(initLight1 && initLightTime.milliseconds() >= 750){
             cmdORANGE();
             initLight1 = false;
             initLightTime.reset();
             initLight2 = true;
         }

         if(initLight2 && initLightTime.milliseconds() >= 750){
             cmdOFF();
             initLight2 = false;
             initLightTime.reset();
             initLight1 = true;
         }

     }

    /**
     * User defined start method.
     * <p>
     * This method will be called once when the PLAY button is first pressed.
     * This method is optional. By default this method takes not action.
     * Example usage: Starting another thread.
     */
    public void start(){

        initLight1 = false;
        initLight2 = false;
        cmdRED();

    }

    /**
     * User defined loop method
     * <p>
     * This method will be called repeatedly in a loop while this op mode is running
     */
    public void loop(){
    ReturnToBaseColor();
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

    public void cmdRED(){
        PeaLight.setPosition(Red);
        CurrentColor = Color.RED;

    }

    public void cmdGREEN(){
        PeaLight.setPosition(Green);
        CurrentColor = Color.GREEN;

    }

    public void cmdYELLOW(){
        PeaLight.setPosition(Yellow);
        CurrentColor = Color.YELLOW;

    }

    public void cmdPURPLE(){
        PeaLight.setPosition(Purple);
        CurrentColor = Color.PURPLE;

    }

    public void cmdBLUE(){
        PeaLight.setPosition(Blue);
        CurrentColor = Color.BLUE;
    }

    public void cmdORANGE(){
        PeaLight.setPosition(Orange);
        CurrentColor = Color.ORANGE;

    }

    public void cmdOFF(){
        PeaLight.setPosition(Off);
        CurrentColor = Color.OFF;

    }


public void UpdateBaseColor (RevBlinkinLedDriver.BlinkinPattern newColor){
   // pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
    baseColor = newColor;
    blinkinLedDriver.setPattern(baseColor);

}
public void SetTempColor (RevBlinkinLedDriver.BlinkinPattern tempColor){
    blinkinLedDriver.setPattern(tempColor);
    runtime.reset();
}
private void ReturnToBaseColor () {
    if (runtime.milliseconds() > TempColorTimeout) {
        blinkinLedDriver.setPattern(baseColor);
    }

}

    public enum Color {
        GREEN,
        RED,
        YELLOW,
        PURPLE,
        BLUE,
        ORANGE,
        OFF
    }

    }
