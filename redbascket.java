package com.example.meepmeeptesting;


import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepTest {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(690);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(180), Math.toRadians(0), 11)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-33, -62, Math.PI/2))

                .strafeToConstantHeading(new Vector2d(-9, -36))
                .waitSeconds(1)
                .strafeToConstantHeading(new Vector2d(-50, -48))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d( -59, -56), Math.PI/4)
                .waitSeconds(2)
                .strafeToLinearHeading(new Vector2d( -58, -46), Math.PI/2)
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d( -54, -52), Math.PI/2)
                .waitSeconds(2)
                .splineTo(new Vector2d(-28, -8), Math.PI * 0)
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(1f)
                .addEntity(myBot)
                .start();
    }
}
