package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Prism.Color;
import org.firstinspires.ftc.teamcode.util.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.util.Prism.PrismAnimations;


public class Led {

    public static enum State {
        SOLID_RED,
        SOLID_GREEN,
        SOLID_WHITE,
        BLINK_RED,
        BLINK_GREEN
    }

    public GoBildaPrismDriver prism;

    private static final GoBildaPrismDriver.Artboard[] STATE_TO_ARTBOARD = {
            GoBildaPrismDriver.Artboard.ARTBOARD_0,  // SOLID_RED
            GoBildaPrismDriver.Artboard.ARTBOARD_1,  // SOLID_GREEN
            GoBildaPrismDriver.Artboard.ARTBOARD_2,  // SOLID_WHITE
            GoBildaPrismDriver.Artboard.ARTBOARD_3,  // BLINK_RED
            GoBildaPrismDriver.Artboard.ARTBOARD_4   // BLINK_GREEN
    };

    private boolean artboardsInitialized = false;


    public Led(HardwareMap hwMap) {
        prism = hwMap.get(GoBildaPrismDriver.class, "led");
        prism.initialize();
        prism.enableDefaultBootArtboard(false);
        prism.clearAllAnimations();
    }

    public void initializeArtboards() {
        PrismAnimations.Solid red = new PrismAnimations.Solid(Color.RED);
        red.setBrightness(100);
        prism.clearAllAnimations();
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, red);
        prism.saveCurrentAnimationsToArtboard(STATE_TO_ARTBOARD[0]);

        PrismAnimations.Solid green = new PrismAnimations.Solid(Color.GREEN);
        green.setBrightness(100);
        prism.clearAllAnimations();
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, green);
        prism.saveCurrentAnimationsToArtboard(STATE_TO_ARTBOARD[1]);

        PrismAnimations.Solid white = new PrismAnimations.Solid(Color.WHITE);
        white.setBrightness(100);
        prism.clearAllAnimations();
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, white);
        prism.saveCurrentAnimationsToArtboard(STATE_TO_ARTBOARD[2]);

        PrismAnimations.Blink blinkRed = new PrismAnimations.Blink();
        blinkRed.setPrimaryColor(Color.RED);
        blinkRed.setSecondaryColor(Color.TRANSPARENT);
        blinkRed.setPeriod(200);
        blinkRed.setPrimaryColorPeriod(100);
        blinkRed.setBrightness(100);
        prism.clearAllAnimations();
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, blinkRed);
        prism.saveCurrentAnimationsToArtboard(STATE_TO_ARTBOARD[3]);

        PrismAnimations.Blink blinkGreen = new PrismAnimations.Blink();
        blinkGreen.setPrimaryColor(Color.GREEN);
        blinkGreen.setSecondaryColor(Color.TRANSPARENT);
        blinkGreen.setPeriod(200);
        blinkGreen.setPrimaryColorPeriod(100);
        blinkGreen.setBrightness(100);
        prism.clearAllAnimations();
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, blinkGreen);
        prism.saveCurrentAnimationsToArtboard(STATE_TO_ARTBOARD[4]);

        artboardsInitialized = true;
        prism.clearAllAnimations();
    }

    public void setState(State state) {
        if (!artboardsInitialized) {
            setStateDirectly(state);
        } else {
            prism.loadAnimationsFromArtboard(STATE_TO_ARTBOARD[state.ordinal()]);
        }
    }

    private void setStateDirectly(State state) {
        prism.clearAllAnimations();

        PrismAnimations.Solid solid;
        PrismAnimations.Blink blink;

        switch (state) {
            case SOLID_RED:
                solid = new PrismAnimations.Solid(Color.RED);
                solid.setBrightness(100);
                prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, solid);
                break;
            case SOLID_GREEN:
                solid = new PrismAnimations.Solid(Color.GREEN);
                solid.setBrightness(100);
                prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, solid);
                break;
            case SOLID_WHITE:
                solid = new PrismAnimations.Solid(Color.WHITE);
                solid.setBrightness(100);
                prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, solid);
                break;
            case BLINK_RED:
                blink = new PrismAnimations.Blink();
                blink.setPrimaryColor(Color.RED);
                blink.setSecondaryColor(Color.TRANSPARENT);
                blink.setPeriod(200);
                blink.setPrimaryColorPeriod(100);
                blink.setBrightness(100);
                prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, blink);
                break;
            case BLINK_GREEN:
                blink = new PrismAnimations.Blink();
                blink.setPrimaryColor(Color.GREEN);
                blink.setSecondaryColor(Color.TRANSPARENT);
                blink.setPeriod(200);
                blink.setPrimaryColorPeriod(100);
                blink.setBrightness(100);
                prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, blink);
                break;
        }
    }

}