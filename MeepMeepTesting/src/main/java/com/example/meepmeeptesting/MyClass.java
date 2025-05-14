package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.MinMax;
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
                .setConstraints(50, 50, Math.PI, Math.PI, 11.35)
                .setDimensions(14.858, 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-47+7.5, -72+7.4, Math.toRadians(0)))
                        .setTangent(Math.toRadians(130))
                        .splineToSplineHeading(new Pose2d(-56, -56, Math.toRadians(45)), Math.toRadians(140))
                        .setTangent(Math.toRadians(45))
                        .splineToSplineHeading(new Pose2d(-48, -42, Math.toRadians(90)), Math.toRadians(90))
                        .setTangent(Math.toRadians(-90))
                        .splineToSplineHeading(new Pose2d(-56, -56, Math.toRadians(45)), Math.toRadians(-135))
                .setTangent(Math.toRadians(110))
                .splineToSplineHeading(new Pose2d(-58, -42, Math.toRadians(90)), Math.toRadians(90))
                .setTangent(Math.toRadians(-30))
                .splineToSplineHeading(new Pose2d(-56, -56, Math.toRadians(45)), Math.toRadians(-135))
                        .setTangent(Math.toRadians(70))
                        .splineToSplineHeading(new Pose2d(-24, -11, Math.toRadians(0)), Math.toRadians(0))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(1f)
                .addEntity(myBot)
                .start();
    }
}