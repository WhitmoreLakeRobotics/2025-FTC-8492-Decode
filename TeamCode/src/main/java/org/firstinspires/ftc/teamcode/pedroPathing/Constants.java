package org.firstinspires.ftc.teamcode.pedroPathing;

import android.util.Log;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.OTOSConstants;
import com.pedropathing.ftc.localization.localizers.OTOSLocalizer;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import kotlinx.coroutines.scheduling.CoroutineScheduler;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .forwardZeroPowerAcceleration(39.31817) //changed from (-35.4224625953)
            .lateralZeroPowerAcceleration(65.39333) ///chabged from (-43.517320985)
// org .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.008, 0.01))
            .translationalPIDFCoefficients(new PIDFCoefficients(0.07, 0, 0.003, 0.025))
//org            .headingPIDFCoefficients(new PIDFCoefficients(0.8, 0, 0.01, 0.025))
            .headingPIDFCoefficients(new PIDFCoefficients(0.6, 0.003, 0.00, 0.025))
           // .useSecondaryTranslationalPIDF(true)
           // .useSecondaryHeadingPIDF(true)
           // .useSecondaryDrivePIDF(true)
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.025,0,.00001,0,0.01))
            .centripetalScaling(.0005)
            .mass(2.72);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1) //this should be at max for tuning
            .rightFrontMotorName("RDM1")
            .rightRearMotorName("RDM2")
            .leftRearMotorName("LDM2")
            .leftFrontMotorName("LDM1")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(71.977) //changedd from (72.38418)
            .yVelocity(63.43330924);

public SparkFunOTOS.Pose2D myOffset =new SparkFunOTOS.Pose2D(6.0,-0.1875,Math.toRadians(90));
 //   public Pose myOffset = new Pose(6.0, -0.1875, Math.toRadians(180));
    public static OTOSConstants localizerConstants =  new OTOSConstants()
            .hardwareMapName("otto")
            .linearUnit(DistanceUnit.INCH)
            .angleUnit(AngleUnit.RADIANS)
            .offset(new SparkFunOTOS.Pose2D(6.0, -0.1875, 4.7124))
//            .offset(myOffset)
            .linearScalar(1.081984) //Multiplier
            .angularScalar(0.9689) ;//Multiplier




    public static PathConstraints pathConstraints = new PathConstraints(0.99, 50, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .OTOSLocalizer(localizerConstants)
                .build();
    }

}
