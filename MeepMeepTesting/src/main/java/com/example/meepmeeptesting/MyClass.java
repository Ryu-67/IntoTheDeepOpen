package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MyClass {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(65, 48, Math.PI*2, Math.PI, 11.35)
                .setDimensions(14.858, 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(7.4, -63.5, Math.toRadians(-90)))
                .setTangent(-Math.toRadians(90))
                .lineToY(-35)
                .waitSeconds(0.5)
                .setTangent(Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(26, -38, Math.toRadians(26)), Math.toRadians(26))
                .waitSeconds(1)
                .turn(Math.toRadians(-65))
                .waitSeconds(0.35)
                .setTangent(0)
                .lineToXSplineHeading(34, Math.toRadians(24))
                .waitSeconds(0.35)
                .turn(-Math.toRadians(65))
                .waitSeconds(0.35)
                .setTangent(0)
                .lineToXSplineHeading(43, Math.toRadians(22))
                .waitSeconds(0.35)
                .setTangent(Math.toRadians(-45))
                .splineToLinearHeading(new Pose2d(42, -50, Math.toRadians(-90)), Math.toRadians(-90))
                .setTangent(Math.toRadians(-90))
                .lineToY(-55)
                .waitSeconds(0.7)
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(10, -35), Math.toRadians(90))
                .waitSeconds(0.5)
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(37, -50, Math.toRadians(-45)), Math.toRadians(-45))
                .waitSeconds(0.3)
                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(new Pose2d(0, -35, Math.toRadians(-90)), Math.toRadians(90))
                .waitSeconds(0.5)
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(37, -50, Math.toRadians(-45)), Math.toRadians(-45))
                .waitSeconds(0.3)
                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(new Pose2d(4, -35, Math.toRadians(-90)), Math.toRadians(90))
                .waitSeconds(0.5)

//                .setTangent(Math.toRadians(90))
//                .splineToLinearHeading(new Pose2d(-59, -59, Math.toRadians(45)), Math.toRadians(225))
//                .waitSeconds(1.5)
////                .setTangent(Math.toRadians(45))
////                .splineToLinearHeading(new Pose2d(14, -60, 0), 0)
////                .waitSeconds(0.35)
////                .setTangent(Math.PI)
////                .splineToSplineHeading(new Pose2d(-59, -59, Math.toRadians(45)), Math.toRadians(-135))
////                        .waitSeconds(1.25)
//                .setTangent(Math.toRadians(45))
//                .splineToLinearHeading(new Pose2d(-48.5, -41, Math.toRadians(90)), Math.PI/2)
//                .waitSeconds(1.5)
//                .setTangent(Math.toRadians(-90))
//                .splineToLinearHeading(new Pose2d(-58.25, -58.25, Math.toRadians(45)), Math.toRadians(225))
//                .waitSeconds(1.5)
//                .setTangent(Math.toRadians(45))
//                .splineToLinearHeading(new Pose2d(-58, -41, Math.toRadians(90)), Math.PI/2)
//                .waitSeconds(1.5)
//                .setTangent(Math.toRadians(-90))
//                .splineToLinearHeading(new Pose2d(-59.5, -59.5, Math.toRadians(45)), Math.toRadians(225))
//                .waitSeconds(1.5)
//                .splineToLinearHeading(new Pose2d(-58.65, -36.25, Math.toRadians(140)), Math.toRadians(135))
//                .waitSeconds(1.5)
//                .setTangent(Math.toRadians(-90))
//                .splineToLinearHeading(new Pose2d(-57.8, -57.8, Math.toRadians(45)), Math.toRadians(225))
//                .waitSeconds(1.5)
//                .setTangent(Math.toRadians(45))
//                .splineToLinearHeading(new Pose2d(-33, -13, Math.toRadians(0)), 0)

                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(1f)
                .addEntity(myBot)
                .start();
    }
}