package org.firstinspires.ftc.teamcode.DisabledAutons;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Common.Settings;
import org.firstinspires.ftc.teamcode.Hardware.Robot;

@Disabled
@Autonomous(name = "dTwoCycleFarRedi2", group = "Auton")
// @Autonomous(...) is the other common choice

public class dTwoCycleFarRedi2 extends OpMode {

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

            case _00_preStart:
                currentStage = stage._05_ForwardStart;
                break;


            case _05_ForwardStart:
                if (robot.driveTrain.getCmdComplete())     {
                    robot.driveTrain.CmdDrive(4,0,0.15,0);
                    currentStage = stage._10_PreLaunch;
                }


                break;
            case _10_PreLaunch:
                if (robot.driveTrain.getCmdComplete()) {
                    robot.driveTrain.CmdDrive(0, 0, 0.0, 0);
                    robot.launcher.cmdOutfar();
                    runtime.reset();
                    currentStage = stage._20_Launch;
                }

                break;
            case _20_Launch:
                if(runtime.milliseconds() >=1500){
                    robot.driveTrain.CmdDrive(0,0,0.0,0);
                    robot.launcherBlocker.cmdUnBlock();
                    robot.transitionRoller.cmdSpin();
                    robot.intake.cmdFoward();
                    runtime.reset();
                    currentStage = stage._25_StopLaunch;

                }

                break;
            case _25_StopLaunch:
                if (runtime.milliseconds() >=2500)     {
                    robot.driveTrain.CmdDrive(0,0,0.0,0);
                    robot.launcherBlocker.cmdBlock();
                    robot.launcher.cmdStop();
                    runtime.reset();
                    currentStage = stage._30_MoveForward;
                }

                break;
            case _30_MoveForward:
                if (runtime.milliseconds() >=500)     {
                    robot.driveTrain.CmdDrive(10,0,0.35,0);
                    currentStage = stage._40_TurnRight1;
                }

                break;
            case _40_TurnRight1:
                if (robot.driveTrain.getCmdComplete())     {
                    robot.driveTrain.cmdTurn(65,0.25);
                    currentStage = stage._50_MoveForward2;
                }

                break;
            case _50_MoveForward2:
                if (robot.driveTrain.getCmdComplete())     {
                    robot.driveTrain.CmdDrive(28,65,0.20,65);
                    currentStage = stage._60_MoveBack;
                }

                break;
            case _60_MoveBack:
                if (robot.driveTrain.getCmdComplete())     {
                    robot.driveTrain.CmdDrive(28,245,0.20,65);   //bearing possibly -240
                    currentStage = stage._70_TurnLeft1;
                }

                break;
            case _70_TurnLeft1:
                if (robot.driveTrain.getCmdComplete())     {
                    robot.driveTrain.cmdTurn(-2,0.25);
                    robot.transitionRoller.cmdStop();
                    currentStage = stage._75_MoveBack2;
                }

                break;
            case _75_MoveBack2:
                if (robot.driveTrain.getCmdComplete())     {
                    robot.driveTrain.CmdDrive(7,180,0.35,-2);
                    currentStage = stage._80_PreLaunch2;
                }

                break;
            case _80_PreLaunch2:
                if(robot.driveTrain.getCmdComplete()){
                    robot.driveTrain.CmdDrive(0,0,0.0,-2);
                    robot.launcher.cmdOutfar();
                    runtime.reset();
                    currentStage = stage._90_Launch2;

                }

                break;
            case _90_Launch2:
                if(runtime.milliseconds() >=1500){
                    robot.driveTrain.CmdDrive(0,0,0.0,-2);
                    robot.launcherBlocker.cmdUnBlock();
                    robot.transitionRoller.cmdSpin();
                    runtime.reset();
                    currentStage = stage._100_StopLaunch2;

                }

                break;
            case _100_StopLaunch2:
                if (runtime.milliseconds() >=2500)     {
                    robot.driveTrain.CmdDrive(0,0,0.0,-2);
                    robot.launcherBlocker.cmdBlock();
                    robot.launcher.cmdStop();
                    runtime.reset();
                    currentStage = stage._102_Backup;
                }

                break;
            case _102_Backup:
                if (runtime.milliseconds() >= 500)     {
                    robot.driveTrain.CmdDrive(8,180,0.35,-2);
                    currentStage = stage._103_TurnLeft2;
                }

                break;
            case _103_TurnLeft2:
                if (robot.driveTrain.getCmdComplete())      {
                    robot.driveTrain.cmdTurn(70,0.35);
                    robot.intake.cmdFoward();
                    currentStage = stage._105_MoveForward3;
                }


                break;
            case _105_MoveForward3:
                if (robot.driveTrain.getCmdComplete())     {
                    robot.driveTrain.CmdDrive(35,70,0.35,70);
                    currentStage = stage._107_ResetGyro;
                }

                break;
            case _107_ResetGyro:
                if (robot.driveTrain.getCmdComplete())     {
                    robot.driveTrain.ResetGyro();
                    currentStage = stage._110_End;
                }

                break;

            case _110_End:
                if(robot.driveTrain.getCmdComplete()){
                    robot.stop();


                }







                break;
        }



    }  //  loop


    //Code to run ONCE after the driver hits STOP

    @Override
    public void stop() {
        robot.stop();
    }

    private enum stage {
        _unknown,
        _00_preStart,
        _05_ForwardStart,
        _10_PreLaunch,
        _20_Launch,
        _25_StopLaunch,
        _30_MoveForward,
        _40_TurnRight1,
        _50_MoveForward2,
        _60_MoveBack,
        _70_TurnLeft1,
        _75_MoveBack2,
        _80_PreLaunch2,
        _90_Launch2,
        _100_StopLaunch2,
        _102_Backup,
        _103_TurnLeft2,
        _105_MoveForward3,
        _107_ResetGyro,
        _110_End


    }
}

