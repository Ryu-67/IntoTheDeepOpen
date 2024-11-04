package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MyClass {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 40, Math.PI*2, Math.PI, 11.35)
                .setDimensions(14.8, 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-39, -64.5, Math.toRadians(90)))
                .splineToLinearHeading(new Pose2d(-59, -59, Math.toRadians(45)), Math.PI)
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(-49.5, -40.8, Math.toRadians(90)), Math.PI/2)
                        .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(-59, -59, Math.toRadians(45)), Math.PI)
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(-59, -50, Math.toRadians(90)), Math.PI/2)
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(-59, -59, Math.toRadians(45)), Math.PI)
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(-50, -48, Math.toRadians(128)), Math.PI/2)
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(-59, -59, Math.toRadians(45)), Math.PI*1.5)
                .waitSeconds(1)
                        .splineToLinearHeading(new Pose2d(-26, -12, Math.toRadians(0)), 0)
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}