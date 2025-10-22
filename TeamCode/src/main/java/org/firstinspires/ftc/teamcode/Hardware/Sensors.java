package org.firstinspires.ftc.teamcode.Hardware;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Base class for FTC Team 8492 defined hardware
 */
public class Sensors extends BaseHardware {

    private ElapsedTime runtime = new ElapsedTime();
    /**
     * The {@link #telemetry} field contains an object in which a user may accumulate data which
     * is to be transmitted to the driver station. This data is automatically transmitted to the
     * driver station on a regular, periodic basis.
     */
    public Telemetry telemetry = null;
    //private ColorRangeSensor IntakeSensor;
    //private DistanceSensor RearLeftSensor



    private ColorSensor SDC01;
    private ColorSensor SDC02;
    private ColorSensor SDC03;
    private ColorSensor NTKC01;





    private boolean cmdComplete = true;
    private Mode CurrentMode = Mode.STOP;

    private int SensorBlue;
    private int SensorRed;
    private int SensorGreen;

    public Target currentTarget = Target.UNKNOWNT;
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
        //DeliverySensor = hardwareMap.get(ColorSensor.class, "DeliveryS");




        SDC01 = hardwareMap.get(ColorSensor.class, "SDC01");
        SDC02 = hardwareMap.get(ColorSensor.class, "SDC02");
        SDC03 = hardwareMap.get(ColorSensor.class, "SDC03");
        NTKC01 = hardwareMap.get(ColorSensor.class, "NTKC01");





    }

    /**
     * User defined init_loop method
     * <p>
     * This method will be called repeatedly when the INIT button is pressed.
     * This method is optional. By default this method takes no action.
     */
    public void init_loop() {

        int red1 = SDC01.red();
        int green1 = SDC01.green();
        int blue1 = SDC01.blue();

        int red2 = SDC02.red();
        int green2 = SDC02.green();
        int blue2 = SDC02.blue();

        int red3 = SDC03.red();
        int green3 = SDC03.green();
        int blue3 = SDC03.blue();

        int red4 = NTKC01.red();
        int green4 = NTKC01.green();
        int blue4 = NTKC01.blue();



        telemetry.addData("Blue", SDC01.blue());
        telemetry.addData("Red ",SDC01.red());
        telemetry.addData("Green ",SDC01.green());


        telemetry.addData("Blue", SDC02.blue());
        telemetry.addData("Red ",SDC02.red());
        telemetry.addData("Green ",SDC02.green());


        telemetry.addData("Blue", SDC03.blue());
        telemetry.addData("Red ",SDC03.red());
        telemetry.addData("Green ",SDC03.green());


        telemetry.addData("Blue", NTKC01.blue());
        telemetry.addData("Red ",NTKC01.red());
        telemetry.addData("Green ",NTKC01.green());

        /**
         * User defined init_loop method
         * <p>
         * This method will be called repeatedly when the INIT button is pressed.
         * This method is optional. By default this method takes no action.
         */

//         telemetry.addData("FLDS1 Pos " , FLDS1.getDistance(DistanceUnit.INCH)) ;
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
    /*
    public void CheckForTarget() {
        updateColorSensor();

        Color Target;
        if ((Target.GREENT.red <= SensorRed) &&
                (Target.GREENT.blue >= SensorBlue) &&
                (Target.GREENT.green >= SensorGreen)) {
            currentTarget = Target.GREENT;
        } else if ((Target.PURPLET.red >= SensorRed) &&      <-------FIX PLEASE!
                (Target.PURPLET.blue <= SensorBlue) &&
                (Target.PURPLET.green >= SensorGreen)) {
            currentTarget = Target.PURPLET;
        } else {
            currentTarget = Target.UNKNOWNT;

        }

    }
*/

    public void doStop(){
        CurrentMode = Mode.STOP;
        cmdComplete = true;
    }



    /**
     * User defined stop method
     * <p>
     * This method will be called when this op mode is first disabled
     * <p>
     * The stop method is optional. By default this method takes no action.
     */



public void stop(){

}


public enum Mode{
    STOP
}


    public enum Target {
        GREENT(60, 90, 70),
        PURPLET(110, 60, 90),
        UNKNOWNT(10, 10, 10);



        private int red;
        private int blue;
        private int green;

        Target(int red, int blue, int green) {
            this.red = red;
            this.blue = blue;
            this.green = green;
        }

        }



        private void updateColorSensor() {
        }




}


