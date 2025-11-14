package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.hardware.limelightvision;
//import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

/**
 * Base class for FTC Team 8492 defined hardware
 */
@Disabled
public class Limey extends BaseHardware {

    private ElapsedTime runtime = new ElapsedTime();
    /**
     * The {@link #telemetry} field contains an object in which a user may accumulate data which
     * is to be transmitted to the driver station. This data is automatically transmitted to the
     * driver station on a regular, periodic basis.
     */
    public Telemetry telemetry = null;
    //private ColorRangeSensor IntakeSensor;
    //private DistanceSensor RearLeftSensor

    private com.qualcomm.hardware.limelightvision Limelight3A;

    private Limelight3A Limey;
    private LLStatus status;
    private LLResult result;
    private int AprilTagID;
    private double ty;
    private double tx;
    private List<List<Double>> corners;
   private Pose3D TagPose;
    private double TagAngle;
    private double TagDistance;
    


    private boolean cmdComplete = true;
    private Mode CurrentMode = Mode.STOP;



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

        Limelight3A Limey  = hardwareMap.get(Limelight3A.class, "limelight");

        //telemetry.setMsTransmissionInterval(11);

        Limey.pipelineSwitch(1);


    }

    /**
     * User defined init_loop method
     * <p>
     * This method will be called repeatedly when the INIT button is pressed.
     * This method is optional. By default this method takes no action.
     */
    public void init_loop() {



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
        Limey.start();

    }

    /**
     * User defined loop method
     * <p>
     * This method will be called repeatedly in a loop while this op mode is running
     */
    public void loop(){
        status = Limey.getStatus();
        result = Limey.getLatestResult();

        if(result.isValid()) {
            AprilTagID = result.getFiducialResults().get(0).getFiducialId();
            ty = result.getFiducialResults().get(0).getTargetYDegrees();
            tx = result.getFiducialResults().get(0).getTargetXDegrees();
            //corners = result.getFiducialResults().get(0).getTargetCorners();
            TagPose = result.getFiducialResults().get(0).getTargetPoseCameraSpace();
            TagAngle = TagPose.getOrientation().getYaw();
            TagDistance = TagPose.getPosition().z;

        }else {
            AprilTagID = -1;
        }
    }

    public  double getTy(){
        return  ty;
    }

    public  double getTx(){
        return  tx;
    }
    public double getTagID(){
     return AprilTagID;
    }

    public double getTagDistance(){
        return TagDistance;
    }

    public double getTagAngle(){
        return TagAngle;
    }


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


    public enum TargetType {
        GREENT(25, 5, 75,25,1,3 ),
        PURPLET(6, 1, 1, 2, 7,1 ),
        UNKNOWNT(1, 1, 1,1,1,1);



        private int red;
        private int  redTol;
        private int blue;
        private int blueTol;
        private int green;
        private int greenTol;


        TargetType(int red,int redTol,int blue,int blueTol,int green,int greenTol) {
            this.red = red;
            this.redTol = redTol;
            this.blue = blue;
            this.blueTol = blueTol;
            this.green = green;
            this.greenTol = greenTol;
        }

        }








}


