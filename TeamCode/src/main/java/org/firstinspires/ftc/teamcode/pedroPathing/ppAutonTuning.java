package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.teamcode.pedroPathing.CompBotConstants.pathConstraints;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.OTOSConstants;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Common.CommonLogic;
import org.firstinspires.ftc.teamcode.Common.Settings;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
@Configurable
@Autonomous(name = "ppAutonTuning", group = "Auton")
// @Autonomous(...) is the other common choice
public class ppAutonTuning extends OpMode{


        //RobotComp robot = new RobotComp();
    //    Robot robot = new Robot();
        private stage currentStage = stage._unknown;


        private String RTAG = "8492-Auton";
// Set up stuff for pedro path

        private String thisUpdate = "11";
        private TelemetryManager telemetryMU;
        private Follower follower;
        private Timer pathTimer, actionTimer, opmodeTimer;
        private ElapsedTime pTimer;// this is for pausing at the end of a path
        //configurables for pedro
        public static int xTol = 2;  // tolorance for x axis in inches
        public static int yTol = 2; // tolorance for y axis in inches

        public static double wallScoreH = Math.toRadians(170);// Heading value for scoring pose near wall
    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));
    private final Pose interPose = new Pose(24, -24, Math.toRadians(90));
    private final Pose endPose = new Pose(24, 24, Math.toRadians(45));
    private Pose currentTargetPose = new Pose(0,0,0);
        // poses for pedropath
       private PathChain path1, path2, path3;//, grabPickup2, scorePickup2, grabPickup3, scorePickup3;

        // private Path grabPickup1a;
        public void buildPaths() {
            /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */


    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */


            /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
            path1 = follower.pathBuilder()
                    .addPath(new BezierLine(startPose, interPose))
                    .setLinearHeadingInterpolation(startPose.getHeading(), interPose.getHeading())
                    .build();
            path2 = follower.pathBuilder()
                    .addPath(new BezierLine(interPose, endPose))
                    .setLinearHeadingInterpolation(interPose.getHeading(), endPose.getHeading())
                    .build();

            /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        path3 = follower.pathBuilder()
                .addPath(new BezierLine(endPose, startPose))
                .setLinearHeadingInterpolation(endPose.getHeading(), startPose.getHeading())
                .build();

        }

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
/*
            robot.hardwareMap = hardwareMap;
            robot.telemetry = telemetry;
            robot.init();*/
            telemetry.addData("Test Auton", "Initialized");

            //Initialize Gyro
            //robot.driveTrain.ResetGyro();
            pathTimer = new Timer();
            opmodeTimer = new Timer();
            opmodeTimer.resetTimer();
            pTimer = new ElapsedTime();

            follower =  createFollower(hardwareMap);
            buildPaths();
            follower.setStartingPose(startPose);
            follower.update();
            //  pedroPanelsTelemetry.init();
            Drawing.init();
            telemetryMU = PanelsTelemetry.INSTANCE.getTelemetry();

            // disp[lay starting postition
            telemetryMU.addData("initialized postition - Update ", thisUpdate);
            // Feedback to Driver Hub for debugging
            updateTelemetry();

        }


        //Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY

        @Override
        public void init_loop() {
            // initialize robot
            //robot.init_loop();

        }


        //Code to run ONCE when the driver hits PLAY

        @Override
        public void start() {
            // start robot
            runtime.reset();
           // robot.start();
            opmodeTimer.resetTimer();


        }


        //Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP

        @Override
        public void loop() {

            telemetry.addData("Auton_Current_Stage ", currentStage);
            //robot.autonLoop();
            follower.update();
            switch (currentStage) {
                case _unknown:
                    currentStage = stage._00_preStart;
                    break;

                case _00_preStart:
                    currentStage = stage._20_DriveToScore;
                    break;

                case _20_DriveToScore:
                    if (!follower.isBusy()) {
                        follower.followPath(path1, true);
                        currentTargetPose = interPose;
                        // follower.update();
                      //  robot.launcher.cmdOuttouch();
                        currentStage = stage._30_Shoot1; // we don't need to do the turn since heading is adjusted in path
                    }
                    break;

                case _30_Shoot1:
                    telemetryMU.addData("follower busy", follower.isBusy());
                    if (!follower.isBusy()) {
                        runtime.reset();
                        currentStage = stage._40_LauncherStop;
                    }
                    break;

                case _40_LauncherStop:
                    while (runtime.milliseconds() < 3000) {
                        telemetryMU.addLine("waiting 1");
                    }
                        currentStage = stage._50_Pickup1;

                    break;

                case _50_Pickup1:
                    if (!follower.isBusy()) {
                        follower.followPath(path2,  true);
                        currentTargetPose = endPose;
                        currentStage = stage._60_Pickup1a;
                        runtime.reset();
                    }
                    break;



                case _60_Pickup1a:
                    if (!follower.isBusy()) {
                        while (runtime.milliseconds() < 3000) {
                            telemetryMU.addLine("waiting 2");
                        }
                        currentStage = stage._70_ToScorePose;
                    }
                    break;
                case _70_ToScorePose:
                    if(!follower.isBusy()){
                        follower.followPath(path3,true);
                        currentTargetPose = startPose;
                        //robot.launcher.cmdOuttouch();
                        currentStage = stage._80_ScorePickup1;
                    }
                    break;
                case _80_ScorePickup1:
                    if (!follower.isBusy()) {
                        while (runtime.milliseconds() < 3000) {
                            telemetryMU.addLine("waiting 3");
                        }
                        currentStage = stage._20_DriveToScore; //start the loop again

                    }

                case _500_End:
                { //do nothing let the time run out

                }


                break;
            }

            updateTelemetry();
        }  //  loop

        private void updateTelemetry() {
            telemetryMU.addData("Current Stage", currentStage);
            telemetryMU.addData("x", follower.getPose().getX());
            telemetryMU.addData("y", follower.getPose().getY());
            telemetryMU.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
            telemetryMU.addData("Current Target Pose", currentTargetPose);
            telemetryMU.addData("breakingStrength", pathConstraints.getBrakingStrength());
            telemetryMU.addData("breakstart ", pathConstraints.getBrakingStart());
            telemetryMU.addData("drivepid P", follower.constants.coefficientsDrivePIDF.P );
            telemetryMU.addData("drivepid D", follower.constants.coefficientsDrivePIDF.D );
            telemetryMU.addData("drivepid F", follower.constants.coefficientsDrivePIDF.F );
            telemetryMU.addData("CONSTRAINTS", "");
            telemetryMU.addData("Tvalue (% complete)", follower.pathConstraints.getTValueConstraint());
            telemetryMU.addData("Current tValue", follower.getCurrentTValue());
            telemetryMU.addData("Velocity Constraint", follower.pathConstraints.getVelocityConstraint());
            telemetryMU.addData("Current Velocity", follower.getVelocity());
            telemetryMU.addData("Trans constraint", follower.pathConstraints.getTranslationalConstraint());
            // telemetryMU.addData("current Trans", follower.getTranslationalError());
            telemetryMU.addData("Heading Constraint", follower.pathConstraints.getHeadingConstraint());

            telemetryMU.update();
            Drawing.drawDebug(follower);
        }

        //Code to run ONCE after the driver hits STOP

        @Override
        public void stop() {
            //robot.stop();
        }

        private enum stage {
            _unknown,
            _00_preStart,
            _20_DriveToScore,
            _30_Shoot1,
            _40_LauncherStop,
            _50_Pickup1,
            _60_Pickup1a,
            _70_ToScorePose,
            _80_ScorePickup1,
            _500_End;


        }

    public static FollowerConstants followerConstants = new FollowerConstants()
            .forwardZeroPowerAcceleration(-32.48245754) //changed from (-35.4224625953)
            .lateralZeroPowerAcceleration(-83.22727075) ///chabged from (-43.517320985)
            //ORG  // .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.008, 0.01))
            .translationalPIDFCoefficients(new PIDFCoefficients(0.15, 0, 0.05, 0.0))
//org            .headingPIDFCoefficients(new PIDFCoefficients(0.8, 0, 0.01, 0.025))
            .headingPIDFCoefficients(new PIDFCoefficients(1, 0.003, 0.06, 0.025))
            // .useSecondaryTranslationalPIDF(true)
            // .useSecondaryHeadingPIDF(true)
            // .useSecondaryDrivePIDF(true)
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.003,0,.000015,0,0.023))
            .centripetalScaling(.0006)
            .mass(12.7);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1) //this should be 1 for tuning
            .rightFrontMotorName("RDM1")
            .rightRearMotorName("RDM2")
            .leftRearMotorName("LDM2")
            .leftFrontMotorName("LDM1")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .xVelocity(79.29855)
            .yVelocity(63.26871);

    //   public Pose myOffset = new Pose(6.0, -0.1875, Math.toRadians(180));
    public static OTOSConstants localizerConstants =  new OTOSConstants()
            .hardwareMapName("otto")
            .linearUnit(DistanceUnit.INCH)
            .angleUnit(AngleUnit.RADIANS)
            .offset(new SparkFunOTOS.Pose2D(-5.5, -2.0, 0))
//            .offset(myOffset) 2.25 5.5
            .linearScalar(1.1211) //Multiplier
            .angularScalar(0.9915) ;//Multiplier



    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 0.15, 0.40);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .OTOSLocalizer(localizerConstants)
                .build();
    }


}
