package org.firstinspires.ftc.teamcode.Autons;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Common.Settings;
import org.firstinspires.ftc.teamcode.Hardware.Robot;


@Autonomous(name = "BlueNearFourCycle", group = "Auton")
// @Autonomous(...) is the other common choice

public class BlueNearForCycle extends OpMode {

    //RobotComp robot = new RobotComp();
    Robot robot = new Robot();




    private stage currentStage = stage._unknown;
    // declare auton power variables
    //private double AUTO_DRIVE_TURBO_SPEED = DriveTrain.DRIVETRAIN_TURBOSPEED;
    //private double AUTO_DRIVE_SLOW_SPEED = DriveTrain.DRIVETRAIN_SLOWSPEED;
    // private double AUTO_DRIVE_NORMAL_SPEED = DriveTrain.DRIVETRAIN_NORMALSPEED;
    // private double AUTO_TURN_SPEED = DriveTrain.DRIVETRAIN_TURNSPEED;

    private String RTAG = "8492-Auton";

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();


    //Code to run ONCE when the driver hits INIT

    @Override
    public void init() {
        //----------------------------------------------------------------------------------------------
        // These constants manage the duration we allow for callbacks to user code to run for before
        // such code is considered to be stuck (in an infinite loop, or wherever) and consequently
        // the robot controller application is restarted. They SHOULD NOT be modified except as absolutely
        // necessary as poorly chosen values might inadvertently compromise safety.
        //----------------------------------------------------------------------------------------------
        msStuckDetectInit = Settings.msStuckDetectInit;
        msStuckDetectInitLoop = Settings.msStuckDetectInitLoop;
        msStuckDetectStart = Settings.msStuckDetectStart;
        msStuckDetectLoop = Settings.msStuckDetectLoop;
        msStuckDetectStop = Settings.msStuckDetectStop;

        robot.hardwareMap = hardwareMap;
        robot.telemetry = telemetry;
        robot.init();
        telemetry.addData("Test Auton", "Initialized");

        //Initialize Gyro
        robot.driveTrain.ResetGyro();
    }


    //Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY

    @Override
    public void init_loop() {
        // initialize robot
        robot.init_loop();

    }


    //Code to run ONCE when the driver hits PLAY

    @Override
    public void start() {
        // start robot
        runtime.reset();
        robot.start();
    }


    //Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP

    @Override
    public void loop() {

        telemetry.addData("Auton_Current_Stage ", currentStage);
        robot.loop();

        switch (currentStage){
            case  _unknown:
                currentStage = stage._00_preStart;
                break;
            case _00_preStart:
                currentStage = stage._20_DriveBack;
                break;


            case _20_DriveBack:
                if (robot.driveTrain.getCmdComplete())     {
                    robot.driveTrain.CmdDrive(19,180,0.35,0);
                    robot.launcher.cmdOuttouch();
                    currentStage = stage._25_Turn2;
                }

            case _25_Turn2:
                if (robot.driveTrain.getCmdComplete())     {
                    robot.driveTrain.cmdTurn(5,0.25);
                    runtime.reset();
                    currentStage = stage._30_Shoot1;
                }

                break;
            case _30_Shoot1:
                if (runtime.milliseconds() >=1000)  {
                    robot.intake.cmdFoward();
                    robot.transitionRoller.cmdSpin();
                    robot.launcherBlocker.cmdUnBlock();
                    runtime.reset();
                    currentStage = stage._40_LauncherStop;
                }
                break;
            case _40_LauncherStop:
                if (runtime.milliseconds() >=5000){
                    robot.driveTrain.cmdTurn(0,0.25);
                    robot.launcherBlocker.cmdBlock();
                    currentStage = stage._45_Forward2;
                }
                break;
                case _45_Forward2:
                if (robot.driveTrain.getCmdComplete())     {
                    robot.driveTrain.CmdDrive(20,0,0.35,0);
                    currentStage = stage._50_Left1;
                }


                break;


            case _50_Left1:
                if (robot.driveTrain.getCmdComplete())     {
                    robot.driveTrain.CmdDrive(60,-90,0.35,0);
                    currentStage = stage._60_Foward1;
                }
                break;

            case _60_Foward1:
                if (robot.driveTrain.getCmdComplete())    {
                    robot.driveTrain.CmdDrive(16,0,0.20,0);
                    currentStage = stage._70_Backwards1;
            }

                break;

            case _70_Backwards1:
                if (robot.driveTrain.getCmdComplete())    {
                    robot.driveTrain.CmdDrive(16,180,0.35,0);
                    currentStage = stage._80_Right1;
                }
                break;

            case _80_Right1:
                if (robot.driveTrain.getCmdComplete())    {
                    robot.driveTrain.CmdDrive(60,90,0.35,0);
                    currentStage = stage._85_Backward2;
                }
                 break;
            case _85_Backward2:
                if (robot.driveTrain.getCmdComplete())     {
                    robot.driveTrain.CmdDrive(20,180,0.35,0);
                    currentStage = stage._90_Shoot2;
                }
                break;
                case _90_Shoot2:
                if (robot.driveTrain.getCmdComplete())  {
                    robot.launcherBlocker.cmdUnBlock();
                    runtime.reset();
                    currentStage = stage._100_Left2;
                }
                break;

            case _100_Left2:
                if (robot.driveTrain.getCmdComplete())     {
                    robot.driveTrain.CmdDrive(60,-90,0.35,0);
                    currentStage = stage._120_Foward3;

                }
                break;

            case _120_Foward3:
                if (robot.driveTrain.getCmdComplete())    {
                    robot.driveTrain.CmdDrive(16,0,0.20,0);
                    currentStage = stage._130_Backward3;
                }
                break;

            case _130_Backward3:
                if (robot.driveTrain.getCmdComplete())    {
                    robot.driveTrain.CmdDrive(16,180,0.35,0);
                    currentStage = stage._140_Right2;
                }
                break;
            case _140_Right2:
                if (robot.driveTrain.getCmdComplete())    {
                    robot.driveTrain.CmdDrive(60,90,0.35,0);
                    currentStage = stage._150_Shoot3;
                }
                break;
            case _150_Shoot3:
                if (robot.driveTrain.getCmdComplete())   {
                    robot.launcherBlocker.cmdUnBlock();
                    runtime.reset();
                    currentStage = stage._160_LauncherStop2;

                }
                break;
            case _160_LauncherStop2:
                if (runtime.milliseconds() >=5000){
                    robot.driveTrain.cmdTurn(0,0.25);
                    robot.launcherBlocker.cmdBlock();
                    currentStage = stage._170_Left3;
                }
                break;


            case _170_Left3:
                if (robot.driveTrain.getCmdComplete())     {
                    robot.driveTrain.CmdDrive(84,-90,0.35,0);
                    currentStage = stage._180_Foward4;

                }
                break;

            case _190_Backward4:
                if (robot.driveTrain.getCmdComplete())    {
                    robot.driveTrain.CmdDrive(16,0,0.20,0);
                    currentStage = stage._200_Right3;
                }
                break;

            case _200_Right3:
                if (robot.driveTrain.getCmdComplete())    {
                    robot.driveTrain.CmdDrive(84,180,0.35,0);
                    currentStage = stage._210_Shoot4;
                }
                break;
            case _210_Shoot4:
                if (runtime.milliseconds() >=5000)    {
                    robot.launcherBlocker.cmdUnBlock();
                    runtime.reset();
                    currentStage = stage._220_LauncherStop3;
                }
                break;
            case _230_Stop:
                if (robot.driveTrain.getCmdComplete())   {
                    robot.launcherBlocker.cmdUnBlock();
                    runtime.reset();
                    currentStage = stage._300_End;

                }
                break;
            case _300_End:
                if(robot.driveTrain.getCmdComplete()){
                    robot.stop();

                }
                break;

        }



    } //  loop


    //Code to run ONCE after the driver hits STOP

    @Override
    public void stop() {
        robot.stop();
    }

    private enum stage {
        _unknown,
        _00_preStart,
        _10_turn,
        _20_DriveBack,
        _25_Turn2,
        _30_Shoot1,
        _40_LauncherStop,
        _45_Forward2,
        _50_Left1,
        _60_Foward1,
        _70_Backwards1,
        _80_Right1,
        _85_Backward2,
        _90_Shoot2,
        _100_Left2,
        _120_Foward3,
        _130_Backward3,
        _140_Right2,
        _150_Shoot3,
        _160_LauncherStop2,
        _170_Left3,
        _180_Foward4,
        _190_Backward4,
        _200_Right3,
        _210_Shoot4,
        _220_LauncherStop3,
        _230_Stop,
        _300_End



    }
}

