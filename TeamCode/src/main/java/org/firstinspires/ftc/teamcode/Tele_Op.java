//package org.firstinspires.ftc.robotcontroller.external.samples;
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;


import org.firstinspires.ftc.robotcore.internal.camera.delegating.DelegatingCaptureSequence;
import org.firstinspires.ftc.teamcode.Common.CommonLogic;
import org.firstinspires.ftc.teamcode.Common.Settings;

import org.firstinspires.ftc.teamcode.Hardware.Flickiteer;
import org.firstinspires.ftc.teamcode.Hardware.Intake;

import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Hardware.Spindexer;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Tele_Op", group = "TeleOp")
//@Disabled
public class Tele_Op extends OpMode {
    private static final String TAGTeleop = "8492-Teleop";
    //RobotTest robot = new RobotTest();
    Robot robot = new Robot();
    //    // Declare OpMode members.
    private boolean gp1_prev_a = false;
    private boolean gp1_prev_b = false;
    private boolean gp1_prev_x = false;
    private boolean gp1_prev_y = false;
    private boolean gp1_prev_right_bumper = false;
    private boolean gp1_prev_left_bumper = false;
    private boolean gp1_prev_dpad_up = false;
    private boolean gp1_prev_dpad_down = false;
    private boolean gp1_prev_dpad_left = false;
    private boolean gp1_prev_dpad_right = false;
    private boolean gp1_prev_back = false;
    private boolean gp1_prev_start = false;

    private boolean gp2_prev_a = false;
    private boolean gp2_prev_b = false;
    private boolean gp2_prev_x = false;
    private boolean gp2_prev_y = false;
    private boolean gp2_prev_right_bumper = false;
    private boolean gp2_prev_left_bumper = false;
    private boolean gp2_prev_dpad_up = false;
    private boolean gp2_prev_dpad_down = false;
    private boolean gp2_prev_dpad_left = false;
    private boolean gp2_prev_dpad_right = false;
    private boolean gp2_prev_back = false;
    private double LeftMotorPower = 0;
    private double RightMotorPower = 0;
    private boolean gp2_prev_start = false;
    private int tHeading = 0;
    private boolean bAutoTurn = false;

    private ElapsedTime runtime = new ElapsedTime();
    private double HLIW = 500;
    //HowLongItWork


    //*********************************************************************************************

    //Code to run ONCE when the driver hits INIT

    @Override
    public void init() {
        //----------------------------------------------------------------------------------------------
        // Safety Management
        //
        // These constants manage the duration we allow for callbacks to user code to run for before
        // such code is considered to be stuck (in an infinite loop, or wherever) and consequently
        // the robot controller application is restarted. They SHOULD NOT be modified except as absolutely
        // necessary as poorly chosen values might inadvertently compromise safety.
        //----------------------------------------------------------------------------------------------
        // msStuckDetectInit = Settings.msStuckDetectInit;
        // msStuckDetectInitLoop = Settings.msStuckDetectInitLoop;
        //msStuckDetectStart = Settings.msStuckDetectStart;
        //msStuckDetectLoop = Settings.msStuckDetectLoop;
        // msStuckDetectStop = Settings.msStuckDetectStop;

        telemetry.addData("Tele_Op", "Initialized");

        robot.hardwareMap = hardwareMap;
        robot.telemetry = telemetry;
        //robot.driveTrain.setMaxPower(DriveTrain.DRIVETRAIN_NORMALSPEED);
        robot.init();
        robot.driveTrain.ResetGyro();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery


    }

    //*********************************************************************************************

    //Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY

    @Override
    public void init_loop() {
        robot.init_loop();
    }

    //*********************************************************************************************

    //Code to run ONCE when the driver hits PLAY

    @Override
    public void start() {
        Runtime.getRuntime();


        // robot.lighting.UpdateBaseColor(RevBlinkinLedDriver.BlinkinPattern.GOLD);
        //robot.signalSign.doUP();
        //robot.swing_arm_and_lift.SetPOS(Swing_Arm_And_Lift.Mode.PICKUP);
    }

    //*********************************************************************************************

    // Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP

    @Override
    public void loop() {
        robot.loop();
        write2Log();
        tHeading = getTurnDirection();
        if (Math.abs(gamepad1.right_stick_x) > 0.04) {
            bAutoTurn = false;
        }
        if (gamepad1.right_trigger > 0.4) {
            tHeading = robot.driveTrain.getCurrentHeading();
            bAutoTurn = true;
        }

        //***********   Gamepad 1 controls ********
        if (bAutoTurn) {
            if (gamepad1.right_bumper) {
                robot.driveTrain.cmdTeleOp(CommonLogic.joyStickMath(gamepad1.left_stick_y * -1),
                        CommonLogic.joyStickMath(gamepad1.left_stick_x),
                        robot.driveTrain.autoTurn(tHeading), robot.driveTrain.DTrain_FASTSPEED);
            } else if (gamepad1.left_bumper) {
                robot.driveTrain.cmdTeleOp(CommonLogic.joyStickMath(gamepad1.left_stick_y * -1),
                        CommonLogic.joyStickMath(gamepad1.left_stick_x),
                        robot.driveTrain.autoTurn(tHeading), robot.driveTrain.DTrain_SLOWSPEED);


            } else {

                robot.driveTrain.cmdTeleOp(CommonLogic.joyStickMath(gamepad1.left_stick_y * -1),
                        CommonLogic.joyStickMath(gamepad1.left_stick_x),
                        robot.driveTrain.autoTurn(tHeading), robot.driveTrain.DTrain_NORMALSPEED);
            }
        } else {
            if (gamepad1.right_bumper) {
                robot.driveTrain.cmdTeleOp(CommonLogic.joyStickMath(gamepad1.left_stick_y * -1),
                        CommonLogic.joyStickMath(gamepad1.left_stick_x),
                        CommonLogic.joyStickMath(gamepad1.right_stick_x), robot.driveTrain.DTrain_FASTSPEED);
            } else if (gamepad1.left_bumper) {
                robot.driveTrain.cmdTeleOp(CommonLogic.joyStickMath(gamepad1.left_stick_y * -1),
                        CommonLogic.joyStickMath(gamepad1.left_stick_x),
                        CommonLogic.joyStickMath(gamepad1.right_stick_x), robot.driveTrain.DTrain_SLOWSPEED);


            } else {

                robot.driveTrain.cmdTeleOp(CommonLogic.joyStickMath(gamepad1.left_stick_y * -1),
                        CommonLogic.joyStickMath(gamepad1.left_stick_x),
                        CommonLogic.joyStickMath(gamepad1.right_stick_x), robot.driveTrain.DTrain_NORMALSPEED);
            }

        }

        if (Math.abs(gamepad1.right_stick_y) > Settings.JOYSTICK_DEADBAND_STICK) {


        }
        //***********   Pushers
        //if (CommonLogic.oneShot(gamepad1.a, gp1_prev_a)) {
        if (gamepad1.a) {
            //robot.subPushers.cmdMoveAllDown();
            //      robot.cmdStrafeIntake();
            //    robot.lighting.UpdateBaseColor(RevBlinkinLedDriver.BlinkinPattern.CONFETTI);
        }

        if (gamepad1.b) {
            //robot.subPushers.cmdMoveAllUp();
            //  robot.cmdStrafeDelivery();
            //robot.lighting.UpdateBaseColor(RevBlinkinLedDriver.BlinkinPattern.TWINKLES_PARTY_PALETTE);
        }
        if (CommonLogic.oneShot(gamepad1.back, gp1_prev_back)) {
            //Initialize Gyro
            robot.driveTrain.ResetGyro();
            tHeading = 0;
        }

        if (gamepad1.right_trigger > 0.8) {

        }
        if ((gamepad1.right_trigger <= 0.79) && (gamepad1.right_trigger > 0.10)) {

        }
//            robot.sweeper.setCurrentMode(Sweeper.Mode.STOP);

        // Bumpers high and lower Powers for the wheels,
        //if (CommonLogic.oneShot(gamepad1.left_bumper, gp1_prev_left_bumper)) {
        //  robot.driveTrain.setMaxPower(DriveTrain.DRIVETRAIN_SLOWSPEED);
        //}
        if ((gamepad1.left_trigger > .6) && (gamepad1.right_trigger < .6)) {

        } else if ((gamepad1.left_trigger < .6) && (gamepad1.right_trigger > .6)) {

        } else {
            //robot.driveTrain.setMaxPower(DriveTrain.DRIVETRAIN_NORMALSPEED);
        }


        //if (CommonLogic.oneShot(gamepad1.right_bumper, gp1_prev_right_bumper)) {
        //  robot.driveTrain.setMaxPower(DriveTrain.DRIVETRAIN_SLOWSPEED);
        // }
        // if (gamepad1.right_bumper) {
        //  robot.driveTrain.setMaxPower(DriveTrain.DRIVETRAIN_TURBOSPEED);
        // RobotLog.aa(TAGTeleop,"GamepadRB: " + gamepad1.right_bumper);
        //     telemetry.addData (TAGTeleop, "GamepadRB: " + gamepad1.right_bumper);
        //  } else if(gamepad1.right_bumper == false)
        // {
        //  robot.driveTrain.setMaxPower(DriveTrain.DRIVETRAIN_NORMALSPEED);
        //}

        //***********  Grabbers
        if (CommonLogic.oneShot(gamepad1.dpad_right, gp1_prev_dpad_right)) {
            //if (RBTChassis.subGrabbers.getIsUpRight()) {
            //robot.subGrabbers.cmdMoveDownRight();
            //}
            //       robot.driveTrain.cmdTurnByGyroMec(90);
            /*
            if (robot.spindexer.CurrentMode == Spindexer.Mode.SDin1) {
                robot.spindexer.NTK_POSITION_TWO();
            }

            if (robot.spindexer.CurrentMode == Spindexer.Mode.SDin2) {
                robot.spindexer.NTK_POSITION_THREE();
            }

            if (robot.spindexer.CurrentMode == Spindexer.Mode.SDin3) {      <-----------------------------------------------------
                robot.spindexer.NTK_POSITION_ONE();
            }

            if (robot.spindexer.CurrentMode == Spindexer.Mode.SDout1) {
                robot.spindexer.NTK_POSITION_THREE();
            }

            if (robot.spindexer.CurrentMode == Spindexer.Mode.SDout2) {
                robot.spindexer.NTK_POSITION_ONE();
            }

            if (robot.spindexer.CurrentMode == Spindexer.Mode.SDout3) {
                robot.spindexer.NTK_POSITION_TWO();
            }

             */

        }

        if (CommonLogic.oneShot(gamepad1.dpad_up, gp1_prev_dpad_up)) {
            // if (RBTChassis.subGrabbers.getIsDownRight()) {
            //robot.subGrabbers.cmdMoveUpRight();
            //}
            //  robot.driveTrain.cmdTurnByGyroMec(0);
            /*

            if (robot.spindexer.CurrentMode == Spindexer.Mode.SDout1) {
                robot.spindexer.OTK_POSITION_TWO();
            }

            if (robot.spindexer.CurrentMode == Spindexer.Mode.SDout2) {
                robot.spindexer.OTK_POSITION_THREE();
            }

            if (robot.spindexer.CurrentMode == Spindexer.Mode.SDout3) {
                robot.spindexer.OTK_POSITION_ONE();
            }

            if (robot.spindexer.CurrentMode == Spindexer.Mode.SDin1) {
                robot.spindexer.OTK_POSITION_TWO();
            }

            if (robot.spindexer.CurrentMode == Spindexer.Mode.SDin2) {
                robot.spindexer.OTK_POSITION_THREE();
            }

            if (robot.spindexer.CurrentMode == Spindexer.Mode.SDin3) {
                robot.spindexer.OTK_POSITION_ONE();
            }

             */


        }

        if (CommonLogic.oneShot(gamepad1.dpad_left, gp1_prev_dpad_left)) {
            //if (robot.subGrabbers.getIsDownLeft()) {
            //    robot.subGrabbers.cmdMoveUpLeft();
            //}
            // robot.driveTrain.cmdTurnByGyroMec(-90);
            /*

            if (robot.spindexer.CurrentMode == Spindexer.Mode.SDin1) {
                robot.spindexer.NTK_POSITION_THREE();
            }

            if (robot.spindexer.CurrentMode == Spindexer.Mode.SDin2) {
                robot.spindexer.NTK_POSITION_ONE();
            }

            if (robot.spindexer.CurrentMode == Spindexer.Mode.SDin3) {
                robot.spindexer.NTK_POSITION_TWO();
            }

            if (robot.spindexer.CurrentMode == Spindexer.Mode.SDout1) {
                robot.spindexer.NTK_POSITION_TWO();
            }

            if (robot.spindexer.CurrentMode == Spindexer.Mode.SDout2) {
                robot.spindexer.NTK_POSITION_THREE();
            }

            if (robot.spindexer.CurrentMode == Spindexer.Mode.SDout3) {
                robot.spindexer.NTK_POSITION_ONE();
            }

             */

        }

        if (CommonLogic.oneShot(gamepad1.dpad_down, gp1_prev_dpad_down)) {
            //if (robot.subGrabbers.getIsUpLeft()) {
            //    robot.subGrabbers.cmdMoveDownLeft();
            //}
            // robot.driveTrain.cmdTurnByGyroMec(180);
            /*

            if (robot.spindexer.CurrentMode == Spindexer.Mode.SDout1) {
                robot.spindexer.OTK_POSITION_THREE();
            }

            if (robot.spindexer.CurrentMode == Spindexer.Mode.SDout2) {
                robot.spindexer.OTK_POSITION_ONE();
            }

            if (robot.spindexer.CurrentMode == Spindexer.Mode.SDout3) {
                robot.spindexer.OTK_POSITION_TWO();
            }

            if (robot.spindexer.CurrentMode == Spindexer.Mode.SDin1) {
                robot.spindexer.OTK_POSITION_THREE();
            }

            if (robot.spindexer.CurrentMode == Spindexer.Mode.SDin2) {
                robot.spindexer.OTK_POSITION_ONE();
            }

            if (robot.spindexer.CurrentMode == Spindexer.Mode.SDin3) {
                robot.spindexer.OTK_POSITION_TWO();
            }

             */

        }

        //***********   Gamepad 2 controls ********

        // Bumpers close and open the gripper
        if (( gamepad2.left_bumper == true)) {


                  robot.launcherBlocker.cmdUnBlock();


            //   robot.lighting.UpdateBaseColor(RevBlinkinLedDriver.BlinkinPattern.LIME);
/*
            if (robot.launcherBlocker.AtUnBlocked == false) {
                robot.launcherBlocker.cmdUnBlock();
                robot.launcherBlocker.AtUnBlocked = true;
            } else {
                robot.launcherBlocker.cmdBlock();
                robot.launcherBlocker.AtUnBlocked = false;
            }
            */


          /*
            if (gamepad2.left_bumper)
                RobotLog.aa(TAGTeleop, " gp2_prev_left_bumper : " + gp2_prev_left_bumper);


            if (robot.spindexer.CurrentMode == Spindexer.Mode.SDout1) {

                if (robot.flickiteer.CurrentMode == Flickiteer.Mode.Ready) {
                    robot.flickiteer.cmdFire();
                }
                if (robot.flickiteer.CurrentMode == Flickiteer.Mode.Fire) {
                    robot.flickiteer.cmdReady();
                }



            }


            if (robot.spindexer.CurrentMode == Spindexer.Mode.SDout2) {

                if (robot.flickiteer.CurrentMode == Flickiteer.Mode.Ready) {
                    robot.flickiteer.cmdFire();
                }
                if (robot.flickiteer.CurrentMode == Flickiteer.Mode.Fire) {   <--------------------------
                    robot.flickiteer.cmdReady();
                }

            }

            if (robot.spindexer.CurrentMode == Spindexer.Mode.SDout3) {

                if (robot.flickiteer.CurrentMode == Flickiteer.Mode.Ready) {
                    robot.flickiteer.cmdFire();
                }
                if (robot.flickiteer.CurrentMode == Flickiteer.Mode.Fire) {
                    robot.flickiteer.cmdReady();
                }

            }

             */

        }else{
            robot.launcherBlocker.cmdBlock();
        }


        if (CommonLogic.oneShot(gamepad2.right_bumper, gp2_prev_right_bumper)) {
            robot.bCkSenors = false;

        }
        if (gamepad2.right_bumper) {
            LaunchNear();
        }

        if (CommonLogic.oneShot(gamepad2.back, gp2_prev_back == true)) {
            //  robot.lighting.UpdateBaseColor(RevBlinkinLedDriver.BlinkinPattern.TWINKLES_FOREST_PALETTE);
            runtime.reset();
            robot.transitionRoller.cmdBack();
            if(runtime.milliseconds() >= HLIW){

            }

        }else {
            robot.transitionRoller.cmdSpin();
        }

        if (CommonLogic.oneShot(gamepad2.start, gp2_prev_start)) {
            //robot.arm.setWristUp();

        }
        if (gamepad2.start) {
//            robot.cmdExcecuteBumpStack();   // this was SetPOS() not setting the mode
            //          robot.lighting.UpdateBaseColor(RevBlinkinLedDriver.BlinkinPattern.AQUA);

        }


        if (CommonLogic.oneShot(gamepad2.a, gp2_prev_a)) {
            robot.intake.cmdFoward();
            robot.bCkSenors = true;
            robot.transitionRoller.cmdSpin();
        }

        if (CommonLogic.oneShot(gamepad2.b, gp2_prev_b)) {
            robot.intake.cmdStop();
            robot.transitionRoller.cmdStop();
        }

        if (CommonLogic.oneShot(gamepad2.y, gp2_prev_y)) {
//            robot.lighting.UpdateBaseColor(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
            //robot.subExtender.incPositionIndex();
            NoLaunch();
            //robot.spindexer.
        }


        if (CommonLogic.oneShot(gamepad2.x, gp2_prev_x)) {


        if (robot.intake.AtIntakeStop = false) {
            robot.intake.cmdStop();
            robot.intake.AtIntakeStop = true;
        }
        if (robot.intake.AtIntakeStop = true) {
            robot.intake.cmdBackward();
            robot.intake.AtIntakeStop = false;
        }

        //robot.transitionRoller.cmdBack();
    }

        //robot.swing_arm_and_lift.SwingPos(robot.swing_arm_and_lift.LASTSWINGPOSITION + (int)(gamepad2.left_stick_x) * 5);

        if (Math.abs(gamepad2.left_stick_x) > 0.8) {
            //robot.subLifter.stickControl(-gamepad2.left_stick_y);
        }

        if (Math.abs(gamepad2.left_stick_y) > Settings.JOYSTICK_DEADBAND_STICK) {
            //robot.subLifter.stickControl(-gamepad2.left_stick_y);
            //robot.capper.cmdTeleOp((gamepad2.left_stick_y * 0.5) + (gamepad2.right_stick_y * 0.5));

        }

        else{
        }

        //robot.swing_arm_and_lift.LiftPos(robot.swing_arm_and_lift.LASTLIFTPOSITION + (int)(gamepad2.right_stick_y) * 5);

        if (Math.abs(gamepad2.right_stick_y) > Settings.JOYSTICK_DEADBAND_STICK) {
            //robot.subLifter.stickControl(-gamepad2.left_stick_y);
            //robot.capper.cmdTeleOp(gamepad2.right_stick_y * .5);

        }

        if (CommonLogic.oneShot(gamepad2.dpad_up, gp2_prev_dpad_up)) {
        }

        if (CommonLogic.oneShot(gamepad2.dpad_down, gp2_prev_dpad_down)) {
        }
        if (CommonLogic.oneShot(gamepad2.dpad_right, gp2_prev_dpad_right)) {
            //robot.arm.setCurrentMode(Arm.Mode.DELIVER_TO_LOW_CHAMBER);
        //    robot.lighting.UpdateBaseColor(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
        }

        if (CommonLogic.oneShot(gamepad2.dpad_left, gp2_prev_dpad_left)) {
           // robot.arm.setCurrentMode(Arm.Mode.DELIVER_TO_HIGH_CHAMBER);
  //          robot.lighting.UpdateBaseColor(RevBlinkinLedDriver.BlinkinPattern.ORANGE);
        }



        if (gamepad2.right_trigger > 0.8){

            LaunchFar();
            robot.bCkSenors = false;

            }
        if ((gamepad2.right_trigger <= 0.79) && (gamepad2.right_trigger > 0.10)){

        }
        if (gamepad2.left_trigger > 0.8) {
            LaunchTouch();

        }
        if ((gamepad2.left_trigger <= 0.79) && (gamepad2.left_trigger > 0.10)) {
          //  robot.lighting.UpdateBaseColor(RevBlinkinLedDriver.BlinkinPattern.BLUE_VIOLET);
        }

        // Update the previous status for gamepad1
        gp1_prev_a = gamepad1.a;
        gp1_prev_b = gamepad1.b;
        gp1_prev_x = gamepad1.x;
        gp1_prev_y = gamepad1.y;
        gp1_prev_left_bumper = gamepad1.left_bumper;
        gp1_prev_right_bumper = gamepad1.right_bumper;
        gp1_prev_dpad_down = gamepad1.dpad_down;
        gp1_prev_dpad_left = gamepad1.dpad_left;
        gp1_prev_dpad_up = gamepad1.dpad_up;
        gp1_prev_dpad_right = gamepad1.dpad_right;
        gp1_prev_back = gamepad1.back;
        gp1_prev_start = gamepad1.start;

        // Update the previous status for gamepad 2
        gp2_prev_a = gamepad2.a;
        gp2_prev_b = gamepad2.b;
        gp2_prev_x = gamepad2.x;
        gp2_prev_y = gamepad2.y;
        gp2_prev_left_bumper = gamepad2.left_bumper;
        gp2_prev_right_bumper = gamepad2.right_bumper;
        gp2_prev_dpad_down = gamepad2.dpad_down;
        gp2_prev_dpad_left = gamepad2.dpad_left;
        gp2_prev_dpad_up = gamepad2.dpad_up;
        gp2_prev_dpad_right = gamepad2.dpad_right;
        gp2_prev_back = gamepad2.back;
        gp2_prev_start = gamepad2.start;

    }

    //*********************************************************************************************

      //Code to run ONCE after the driver hits STOP

    @Override
    public void stop() {

        robot.stop();

    }

    private int getTurnDirection(){
        boolean a = gamepad1.a;
        boolean b = gamepad1.b;
        boolean x = gamepad1.x;
        boolean y = gamepad1.y;
    if(a){
        bAutoTurn = true;
        if(x){
           return 45;
        }else  if(b){
            return -45;
        }else {
            return 0;
        }
    }
    else if (b){
        bAutoTurn = true;
        if (y){
            return -135;
        }else {
            return -90;
        }
    }
    else if (y){
        bAutoTurn = true;
        if(x){
            return 135;
        }else{
            return -135;
        }
    }
    else if(x){
        bAutoTurn = true;
       return 90;
    }
    else {
        return tHeading;
    }
    }







    //*********************************************************************************************
    private void  write2Log() {

//
//    RobotLog.aa(TAGTeleop, " gp1_prev_a : " + gp1_prev_a);
//    RobotLog.aa(TAGTeleop, " gp1_prev_b : " + gp1_prev_b);
//    RobotLog.aa(TAGTeleop, " gp1_prev_x : " + gp1_prev_x);
//    RobotLog.aa(TAGTeleop, " gp1_prev_y : " + gp1_prev_y);
//    RobotLog.aa(TAGTeleop, " gp1_prev_right_bumper : " + gp1_prev_right_bumper);
//   RobotLog.aa(TAGTeleop, " gp1_prev_left_bumper : " + gp1_prev_left_bumper);
//    RobotLog.aa(TAGTeleop, " gp1_prev_dpad_up : " + gp1_prev_dpad_up);
//    RobotLog.aa(TAGTeleop, " gp1_prev_dpad_down : " + gp1_prev_dpad_down);
//    RobotLog.aa(TAGTeleop, " gp1_prev_dpad_left : " + gp1_prev_dpad_left);
//    RobotLog.aa(TAGTeleop, " gp1_prev_dpad_right : " + gp1_prev_dpad_right);
//
//    RobotLog.aa(TAGTeleop, " gp2_prev_a : " + gp2_prev_a);
//    RobotLog.aa(TAGTeleop, " gp2_prev_b : " + gp2_prev_b);
//    RobotLog.aa(TAGTeleop, " gp2_prev_x : " + gp2_prev_x);
//    RobotLog.aa(TAGTeleop, " gp2_prev_y : " + gp2_prev_y);
//    RobotLog.aa(TAGTeleop, " gp2_prev_right_bumper : " + gp2_prev_right_bumper);
//    RobotLog.aa(TAGTeleop, " gp2_prev_left_bumper : " + gp2_prev_left_bumper);
        //RobotLog.aa(TAGTeleop, " gp2_prev_dpad_up : " + gp2_prev_dpad_up);
        //RobotLog.aa(TAGTeleop, " gp2_prev_dpad_down : " + gp2_prev_dpad_down);
//    RobotLog.aa(TAGTeleop, " gp2_prev_dpad_left : " + gp2_prev_dpad_left);
//    RobotLog.aa(TAGTeleop, " gp2_prev_dpad_right : " + gp2_prev_dpad_right);
//
//

    }

    public void LaunchTouch() {         //wait for launcher to spin up to speed.
        robot.launcher.cmdOuttouch();
        if (robot.launcher.bAtSpeed) {
            if(robot.launcherBlocker.AtUnBlocked == true){
                robot.transitionRoller.cmdSpin();
            }
            if(robot.launcherBlocker.AtUnBlocked == false) {
                robot.transitionRoller.cmdStop();
            }

        }
    }

    public void LaunchNear(){         //wait for launcher to spin up to speed.
        robot.launcher.cmdOutnear();
        if (robot.launcher.bAtSpeed) {
            //if (CommonLogic.oneShot(gamepad2.left_bumper,gp2_prev_left_bumper)){
           // robot.launcherBlocker.cmdUnBlock();
           if(robot.launcherBlocker.AtUnBlocked == true){
                //if (CommonLogic.oneShot(gamepad2.left_bumper,gp2_prev_left_bumper)) {
                    //robot.launcherBlocker.cmdUnBlock();
                   // if (CommonLogic.oneShot(gamepad2.left_bumper,gp2_prev_left_bumper)) {
                        robot.transitionRoller.cmdSpin();
                   // }

               // }
            }
            if(robot.launcherBlocker.AtUnBlocked == false) {
                robot.transitionRoller.cmdStop();
            }
        }
    }

    public void LaunchFar(){          //wait for launcher to spin up to speed.
        robot.launcher.cmdOutfar();
        if (robot.launcher.bAtSpeed){
            if(robot.launcherBlocker.AtUnBlocked == true) {
                // if (CommonLogic.oneShot(gamepad2.left_bumper,gp2_prev_left_bumper)) {
                // robot.launcherBlocker.cmdUnBlock();
                //if (CommonLogic.oneShot(gamepad2.left_bumper, gp2_prev_left_bumper)) {
                robot.transitionRoller.cmdSpin();
                // }
                //  }
            }
            if(robot.launcherBlocker.AtUnBlocked == false) {
                robot.transitionRoller.cmdStop();
            }
            }
        }


    public void NoLaunch(){
        robot.transitionRoller.cmdStop();
        robot.launcherBlocker.cmdBlock();
        robot.launcher.cmdStop();
    }

}

