package com.example.meepmeep;


import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class redsample {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(690);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(180), Math.toRadians(0), 11)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(14, -62,  Math.PI/2))

                .strafeToConstantHeading(new Vector2d(10, -34))
                .waitSeconds(1)
                .strafeToConstantHeading(new Vector2d(27, -34))
                .splineToLinearHeading(new Pose2d(48, -12, Math.PI ),Math.PI * 0)
                .waitSeconds(1)
                .strafeToConstantHeading(new Vector2d(48, -55))
                .waitSeconds(1)
                .strafeToConstantHeading(new Vector2d(48, -19))
                .splineToConstantHeading(new Vector2d(57, -14), Math.PI * 0) // vai pro 2 sample
                .strafeToConstantHeading(new Vector2d(57, -55))//empurra
                .waitSeconds(1)
                .strafeToConstantHeading(new Vector2d(57, -19))
                .splineToConstantHeading(new Vector2d(63, -14), Math.PI * 0)// vai pro 3 sample
                .strafeToConstantHeading(new Vector2d(63, -55)) // empurra
                .waitSeconds(1)
                .strafeToConstantHeading(new Vector2d(60, -20))
                .splineToConstantHeading(new Vector2d(26, -12), Math.PI)// estacionar
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(1f)
                .addEntity(myBot)
                .start();
    }
}
