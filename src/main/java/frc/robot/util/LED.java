package frc.robot.util;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LED extends SubsystemBase{
    private AddressableLED strip;
    private AddressableLEDBuffer buffer;
    private Color[] colors;
    private static ArrayList<LED> instances;
    private int port;
    private EffectInfo effectInfo;

    public enum Effect {
        RACE,FLASH,NONE;
    }

    private class EffectInfo {
        public Effect effect;
        public int duration;
        public Color[] colors;
        public int marker;
        public EffectInfo(Effect effect, int duration, Color[] colors) {
            this.colors = colors;
            this.effect = effect;
            this.duration = duration;
            this.marker = 0;
        }
    }

    public enum Color {
        Red(255, 0, 0),
        Blue(0,0,255),
        Green(0,255,0),
        Black(0,0,0),
        DarkPink(253, 5, 89),
        DarkGreen(5,233,89),
        Lavender(245,100,120),
        DarkBlue(5,141,238);

        int r; int g; int b;

        private Color(int r, int g, int b){
            this.r = r;
            this.g = g;
            this.b = b;
        }
    }

    public static LED getInstance() {
        return getInstance(Constants.LED.DEFAULT_PORT);
    }

    public static LED getInstance(int port) {
        if (instances == null) {
            instances = new ArrayList<LED>();
            instances.add(new LED(port));
            return instances.get(instances.size()-1);
        } else {
            for (LED instance : instances) {
                if (instance.getPort() == port) {
                    return instance;
                }
            }
        }
        instances.add(new LED(port));
        return instances.get(instances.size()-1);
    }

    private LED(int port) {
        this.port = port;
        strip = new AddressableLED(port);
        colors = new Color[Constants.LED.STRIP_LENGTH];
        buffer = new AddressableLEDBuffer(Constants.LED.STRIP_LENGTH);
        effectInfo = new EffectInfo(Effect.NONE, 0, null);
        strip.setLength(buffer.getLength());

        for (int i = 0; i<buffer.getLength(); i++) {
            colors[i] = Color.Black;
        }
    }

    public void setFlash(double duration, Color[] colors) {
        effectInfo = new EffectInfo(Effect.FLASH, (int)(duration*50), colors);
    }

    public void setRace(double duration, Color[] colors) {
        effectInfo = new EffectInfo(Effect.RACE, (int)(duration*50), colors);
    }

    public int getPort() {
        return port;
    }

    @Override
    public void periodic() {
        switch(effectInfo.effect) {
            case FLASH:
                for(int i = 0; i<buffer.getLength(); i++) {
                    colors[i] = effectInfo.colors[effectInfo.marker/effectInfo.duration];
                }
                effectInfo.marker = (effectInfo.marker + 1)%(effectInfo.colors.length*effectInfo.duration);
                break;
            case RACE:
                if (effectInfo.marker == 0) {
                    colors[colors.length - 1] = Color.Black;
                } else {
                    colors[effectInfo.marker - 1] = Color.Black;
                }
                colors[effectInfo.marker] = effectInfo.colors[0];
                effectInfo.marker = (effectInfo.marker + 1)%(colors.length - 1);
                break;
            case NONE:
                break;
        }

        for(int i = 0; i<buffer.getLength(); i++) {
            buffer.setRGB(i, colors[i].r, colors[i].g, colors[i].b);
        }

        strip.setData(buffer);
        strip.start();
    }
}
