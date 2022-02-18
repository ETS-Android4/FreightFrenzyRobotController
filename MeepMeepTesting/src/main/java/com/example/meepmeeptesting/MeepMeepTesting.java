package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        RoadRunnerBotEntity bot = new DefaultBotBuilder(meepMeep)
                .setConstraints(23, 23, Math.toRadians(148.9506), Math.toRadians(148.9506), 8.48)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-32, 62, Math.toRadians(180)))
                                .lineTo(new Vector2d(-12, 24 + 21))
                                .lineTo(new Vector2d(-50, 60))
                                .lineTo(new Vector2d(-62, 36))
                                .build()
                );
        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_OFFICIAL)
                .setDarkMode(true)
                .addEntity(bot)
                .start();
    }
}