package org.firstinspires.ftc.teamcode.pedroPathing;

//public class TestPedroAuton package org.firstinspires.ftc.teamcode.pedroPathing; // make sure this aligns with class location

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Hardware.Launcher;
import org.firstinspires.ftc.teamcode.Hardware.LauncherBlocker;
import org.firstinspires.ftc.teamcode.Hardware.Robot;

@Autonomous(name = "TT mad Test pedro auton", group = "Examples")
public class TTTestPedroAuton extends OpMode {
 //   Robot robot = new Robot();
    Launcher launcher = new Launcher();
    LauncherBlocker launcherBlocker = new LauncherBlocker();
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;
private stage currentStage = stage._00_preStart;

    private final Pose startPose = new Pose(40.5, 135, Math.toRadians(180)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(60, 130, Math.toRadians(135)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose pickup1Pose = new Pose(25, 90, Math.toRadians(200)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup2Pose = new Pose(35, 35, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose pickup3Pose = new Pose(35, 35, Math.toRadians(0)); // Lowest (Third Set) of Artifacts from the Spike Mark.

    private Path scorePreload;
    private PathChain grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3;

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, scorePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup2Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .build();

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, scorePose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup3Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .build();

        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose, scorePose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .build();
    }

    /**
     * This is the main loop of the OpMode, it will run repeatedly after clicking "Play".
     **/
    @Override
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();
//robot.loop();
        launcher.loop();
        launcherBlocker.loop();
        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /**
     * This method is called once at the init of the OpMode.
     **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
   //     robot.hardwareMap = hardwareMap;
     //   robot.telemetry = telemetry;
//.init();
        launcher.hardwareMap = hardwareMap;
        launcher.init();
        launcher.telemetry = telemetry;
        launcherBlocker.hardwareMap = hardwareMap;
        launcherBlocker.init();
        launcher.telemetry = telemetry;



        follower = CompBotConstants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

    }

    /**
     * This method is called continuously after Init while waiting for "play".
     **/
    @Override
    public void init_loop() {
    }

    /**
     * This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system
     **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
 ////       robot.start();
        launcherBlocker.start();
        launcher.start();
    }

    /**
     * We do not use this because everything should automatically disable
     **/
    @Override
    public void stop() {
  //      robot.stop();
        launcher.cmdStop();
    }

    public void autonomousPathUpdate() {
        switch (currentStage) {
            case _unknown:
                currentStage = stage._00_preStart;
                break;
            case _00_preStart:
                currentStage = stage._10_DriveBack;
                break;
            case _10_DriveBack:
                follower.followPath(scorePreload); //drive backwards to launch site.

                currentStage = stage._20_Launch1;
                break;
            case _20_Launch1:
                if (!follower.isBusy()){

                }

            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Score Preload */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup1, true);
                    currentStage= stage._25_DriveToPickup1;
                }
                break;
            case _25_DriveToPickup1:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if (!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup1, true);
                    currentStage= stage._500_End;
                }
                break;
            case _500_End:
  //              robot.stop();
                stop();

                break;
        }
    }

    /**
     * These change the states of the paths and actions. It will also reset the timers of the individual switches
     **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    private enum stage {
        _unknown,
        _00_preStart,
        _10_DriveBack,
        _20_Launch1,
        _25_DriveToPickup1 ,
        _30_Pickup1,
        _40_DriveTo_score,
        _50_Launch2,
        _500_End;
    }

    }

