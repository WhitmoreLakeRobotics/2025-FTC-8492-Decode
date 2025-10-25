package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Common.CommonLogic;

import kotlin.ranges.URangesKt;


/**
 * Base class for FTC Team 8492 defined hardware
 */
public class Launcher extends BaseHardware{


    /**
     * The {@link #telemetry} field contains an object in which a user may accumulate data which
     * is to be transmitted to the driver station. This data is automatically transmitted to the
     * driver station on a regular, periodic basis.
     */
    public Telemetry telemetry = null;

    public HardwareMap hardwareMap = null; // will be set in Child class


    /**
     * BaseHardware constructor
     * <p>
     * The op mode name should be unique. It will be the name displayed on the driver station. If
     * multiple op modes have the same name, only one will be available.
     */


    private DcMotorEx LaunchM01 ;
    private DcMotorEx LaunchM02 ;

    public Mode CurrentMode;

    private double LaunchM01Power;
    private double LaunchM02Power;

    public final double minPower = -1.0;
    public final double maxPower = 1.0;

    public static final double stopSpeed = 0;
    public static final double topSpeednear =  0.5;
    public static final double topSpeedfar =  1;
    public static final double bottomSpeednear = 0.5;
    public static final double bottomSpeedfar = 1;

    private double kP = 0.005;
    private double kI = 0.0001;
    private double kD = 0.001;

    private double targetRPM1 = 0;
    private double targetRPM2 = 0;
    private double lastError = 0;
    private double integralSum = 0;

    private double targetRPM1Tol = 100;
    private double targetRPM2Tol = 100;

    private ElapsedTime timer = new ElapsedTime();

    public boolean bAtSpeed = false;





    //  public static final double snailoutSpeed = -0.25;


    /**
     * User defined init method
     * <p>
     * This method will be called once when the INIT button is pressed.
     */
    public void init() {



        LaunchM02 = hardwareMap.get(DcMotorEx.class, "LaunchM02");
        LaunchM01 = hardwareMap.get(DcMotorEx.class, "LaunchM01");

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
        runPID();

        if ((CommonLogic.inRange(getMotorRPM(LaunchM01),targetRPM1,targetRPM1Tol)) &&
                (CommonLogic.inRange(getMotorRPM(LaunchM02),targetRPM2,targetRPM2Tol))
        ){
            bAtSpeed = true;
        } else {
        bAtSpeed = false;
        }

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

    public void cmdOutnear(){
        CurrentMode = Mode.LaunchMout;
        //LaunchM01.setPower (topSpeednear);
       // LaunchM02.setPower (bottomSpeednear);
        targetRPM1 = 3000;
        targetRPM2 = 4000;

    }

    public void cmdOutfar(){
        CurrentMode = Mode.LaunchMout;
      //  LaunchM01.setPower (topSpeedfar);
       // LaunchM02.setPower (bottomSpeedfar);
        targetRPM1 = 4000;
        targetRPM2 = 5000;

    }



    public void cmdStop(){
        CurrentMode = Mode.LaunchMstop;
        LaunchM01.setPower (stopSpeed);
        LaunchM02.setPower (stopSpeed);


    }

    public double getMotorRPM(DcMotorEx motor){
        double ticksPerRevolution = 28; //update and double check
        double gearRatio = 1.0; //update and double check
        double ticksPerSecond = motor.getVelocity();
        return (ticksPerSecond / ticksPerRevolution) * 60 * gearRatio;
    }

    private double calculatePID(double currentRPM,double targetRPM){
        double error = targetRPM - currentRPM;
        double deltaTime = timer.seconds();
        timer.reset();

        double proportional = kP * error;

        integralSum += error * deltaTime;
        double integral = kI * integralSum;

        double derivative = kD * (error - lastError) / deltaTime;
        lastError = error;

        return proportional + integral + derivative;
    }

    public void runPID(){
        double currentRPM1 = getMotorRPM(LaunchM01);
        double power1 = calculatePID(currentRPM1,targetRPM1);

        double currentRPM2 = getMotorRPM(LaunchM02);
    double power2 = calculatePID(currentRPM2,targetRPM2);

        LaunchM01.setPower(power1);
        LaunchM02.setPower(power2);

        telemetry.addData("Target RPM",targetRPM1);
        telemetry.addData("Current RPM",currentRPM1);
        telemetry.addData("Motor Power",power1);
        //telemetry.update();

        telemetry.addData("Target RPM",targetRPM2);
        telemetry.addData("Current RPM",currentRPM2);
        telemetry.addData("Motor Power",power2);
        telemetry.update();

    }


    public enum Mode {
        LaunchMout,
        LaunchMstop;
    }








}
