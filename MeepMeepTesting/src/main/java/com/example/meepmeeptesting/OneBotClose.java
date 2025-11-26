package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class OneBotClose {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(500);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeRedDark()) // set our bot to be red
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(80, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-10, -14.5, Math.toRadians(180)))
                        .waitSeconds(2.5)
                .strafeToLinearHeading(new Vector2d(-12,-27), Math.toRadians(270)) // go to get first set of artifacts
                .strafeToLinearHeading(new Vector2d(-12,-50), Math.toRadians(270)) // drive into first set of artifacts

                .strafeToLinearHeading(new Vector2d(-44,-28),Math.toRadians(240)) // go to shoot first batch
                        .waitSeconds(2.5)

                .strafeToLinearHeading(new Vector2d(12,-30), Math.toRadians(270)) // go to second set of artifacts
                .strafeToLinearHeading(new Vector2d(12,-50), Math.toRadians(270)) // drive into second set of artifacts

                .turnTo(Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(-2,-58), Math.toRadians(180)) // push classifier gate
                        .waitSeconds(3)

                .strafeToLinearHeading(new Vector2d(-44,-28),Math.toRadians(240)) // go to shoot second batch\
                        .waitSeconds(2.5)

                .strafeToLinearHeading(new Vector2d(35,-30), Math.toRadians(270)) // go to third set of artifacts



                .strafeToLinearHeading(new Vector2d(35,-50), Math.toRadians(270)) // drive into third set of artifacts
                .strafeToLinearHeading(new Vector2d(-44,-28),Math.toRadians(240)) // go to shoot
                        .waitSeconds(2.5)





                .build());







        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}