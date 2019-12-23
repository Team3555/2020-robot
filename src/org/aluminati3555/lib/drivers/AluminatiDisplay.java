package org.aluminati3555.lib.drivers;

import java.util.Arrays;
import java.util.HashMap;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.I2C.Port;

/**
 * This class provides an interface for the REV Robotics Digit display. Some
 * code is taken from the REV Robotics example.
 * 
 * @author Caleb Heydon
 */
public class AluminatiDisplay {
    private static final byte[] OSC = { (byte) 0x21 };
    private static final byte[] BLINK = { (byte) 0x81 };
    private static final byte[] BRIGHT = { (byte) 0xEF };

    @Override
    public String toString() {
        return "[Diplay]";
    }

    /**
     * Creates a string by repeating chars
     */
    private static String repeat(char c, int n) {
        char[] array = new char[2];
        Arrays.fill(array, c);
        return new String(array);
    }

    private I2C i2c;
    private DigitalInput buttonA;
    private DigitalInput buttonB;
    private AnalogInput potentiometer;

    byte[][] characterRegister;
    HashMap<Character, Integer> characterMap;

    /**
     * Returns true if the button is down
     * 
     * @param button
     * @return
     */
    public boolean getButton(Button button) {
        if (button == Button.BUTTON_A) {
            return buttonA.get();
        } else {
            return buttonB.get();
        }
    }

    /**
     * Returns the current potentiometer voltage
     */
    public double getPotentiometerVoltage() {
        return potentiometer.getVoltage();
    }

    /**
     * Writes a byte array to the I2C port and waits 0.01
     * 
     * @param data
     * @return
     */
    private boolean writeBufferDelayed(byte[] data) {
        boolean value = i2c.writeBulk(data);
        Timer.delay(0.01);

        return value;
    }

    /**
     * Writes data to the display
     * 
     * @param data
     */
    private void write(int[] data) {
        byte[] buffer = new byte[10];
        buffer[0] = (byte) (0b0000111100001111);
        buffer[2] = characterRegister[data[3]][0];
        buffer[3] = characterRegister[data[3]][1];
        buffer[4] = characterRegister[data[2]][0];
        buffer[5] = characterRegister[data[2]][1];
        buffer[6] = characterRegister[data[1]][0];
        buffer[7] = characterRegister[data[1]][1];
        buffer[8] = characterRegister[data[0]][0];
        buffer[9] = characterRegister[data[0]][1];

        i2c.writeBulk(buffer);
    }

    /**
     * Clears the display
     */
    public void clear() {
        int[] data = { 36, 36, 36, 36 };
        write(data);
    }

    /**
     * Displays a number
     */
    public void display(Number number) {
        double decimal = number.doubleValue();

        int[] buffer = { 36, 36, 36, 36 };

        int ten = (int) (decimal / 10);
        int one = (int) (decimal % 10);
        int tenth = (int) ((decimal * 10) % 10);
        int hundredth = (int) ((decimal * 100) % 10);

        if (ten != 0) {
            buffer[0] = ten;
        }

        buffer[1] = one;
        buffer[2] = tenth;
        buffer[3] = hundredth;

        write(buffer);
    }

    /**
     * Displays a string
     * 
     * @param string
     */
    public void display(String string) {
        int[] buffer = new int[4];
        string = repeat(' ', Math.max(0, 4 - string.length())) + string.toUpperCase();

        for (int i = 0; i < 4; i++) {
            Integer g = characterMap.get(string.charAt(i));
            if (g == null) {
                g = 36;
            }
            buffer[i] = g;
        }

        write(buffer);
    }

    public AluminatiDisplay() {
        i2c = new I2C(Port.kMXP, 0x70);
        buttonA = new DigitalInput(19);
        buttonB = new DigitalInput(20);
        potentiometer = new AnalogInput(3);

        characterRegister = new byte[37][2];
        characterMap = new HashMap<Character, Integer>();

        characterRegister[0][0] = (byte) 0b00111111;
        characterRegister[9][1] = (byte) 0b00000000; // 0
        characterMap.put('0', 0);
        characterRegister[1][0] = (byte) 0b00000110;
        characterRegister[0][1] = (byte) 0b00000000; // 1
        characterMap.put('1', 1);
        characterRegister[2][0] = (byte) 0b11011011;
        characterRegister[1][1] = (byte) 0b00000000; // 2
        characterMap.put('2', 2);
        characterRegister[3][0] = (byte) 0b11001111;
        characterRegister[2][1] = (byte) 0b00000000; // 3
        characterMap.put('3', 3);
        characterRegister[4][0] = (byte) 0b11100110;
        characterRegister[3][1] = (byte) 0b00000000; // 4
        characterMap.put('4', 4);
        characterRegister[5][0] = (byte) 0b11101101;
        characterRegister[4][1] = (byte) 0b00000000; // 5
        characterMap.put('5', 5);
        characterRegister[6][0] = (byte) 0b11111101;
        characterRegister[5][1] = (byte) 0b00000000; // 6
        characterMap.put('6', 6);
        characterRegister[7][0] = (byte) 0b00000111;
        characterRegister[6][1] = (byte) 0b00000000; // 7
        characterMap.put('7', 7);
        characterRegister[8][0] = (byte) 0b11111111;
        characterRegister[7][1] = (byte) 0b00000000; // 8
        characterMap.put('8', 8);
        characterRegister[9][0] = (byte) 0b11101111;
        characterRegister[8][1] = (byte) 0b00000000; // 9
        characterMap.put('9', 9);

        characterRegister[10][0] = (byte) 0b11110111;
        characterRegister[10][1] = (byte) 0b00000000; // A
        characterMap.put('A', 10);
        characterRegister[11][0] = (byte) 0b10001111;
        characterRegister[11][1] = (byte) 0b00010010; // B
        characterMap.put('B', 11);
        characterRegister[12][0] = (byte) 0b00111001;
        characterRegister[12][1] = (byte) 0b00000000; // C
        characterMap.put('C', 12);
        characterRegister[13][0] = (byte) 0b00001111;
        characterRegister[13][1] = (byte) 0b00010010; // D
        characterMap.put('D', 13);
        characterRegister[14][0] = (byte) 0b11111001;
        characterRegister[14][1] = (byte) 0b00000000; // E
        characterMap.put('E', 14);
        characterRegister[15][0] = (byte) 0b11110001;
        characterRegister[15][1] = (byte) 0b00000000; // F
        characterMap.put('F', 15);
        characterRegister[16][0] = (byte) 0b10111101;
        characterRegister[16][1] = (byte) 0b00000000; // G
        characterMap.put('G', 16);
        characterRegister[17][0] = (byte) 0b11110110;
        characterRegister[17][1] = (byte) 0b00000000; // H
        characterMap.put('H', 17);
        characterRegister[18][0] = (byte) 0b00001001;
        characterRegister[18][1] = (byte) 0b00010010; // I
        characterMap.put('I', 18);
        characterRegister[19][0] = (byte) 0b00011110;
        characterRegister[19][1] = (byte) 0b00000000; // J
        characterMap.put('J', 19);
        characterRegister[20][0] = (byte) 0b01110000;
        characterRegister[20][1] = (byte) 0b00001100; // K
        characterMap.put('K', 20);
        characterRegister[21][0] = (byte) 0b00111000;
        characterRegister[21][1] = (byte) 0b00000000; // L
        characterMap.put('L', 21);
        characterRegister[22][0] = (byte) 0b00110110;
        characterRegister[22][1] = (byte) 0b00000101; // M
        characterMap.put('M', 22);
        characterRegister[23][0] = (byte) 0b00110110;
        characterRegister[23][1] = (byte) 0b00001001; // N
        characterMap.put('N', 23);
        characterRegister[24][0] = (byte) 0b00111111;
        characterRegister[24][1] = (byte) 0b00000000; // O
        characterMap.put('O', 24);
        characterRegister[25][0] = (byte) 0b11110011;
        characterRegister[25][1] = (byte) 0b00000000; // P
        characterMap.put('P', 25);
        characterRegister[26][0] = (byte) 0b00111111;
        characterRegister[26][1] = (byte) 0b00001000; // Q
        characterMap.put('Q', 26);
        characterRegister[27][0] = (byte) 0b11110011;
        characterRegister[27][1] = (byte) 0b00001000; // R
        characterMap.put('R', 27);
        characterRegister[28][0] = (byte) 0b10001101;
        characterRegister[28][1] = (byte) 0b00000001; // S
        characterMap.put('S', 28);
        characterRegister[29][0] = (byte) 0b00000001;
        characterRegister[29][1] = (byte) 0b00010010; // T
        characterMap.put('T', 29);
        characterRegister[30][0] = (byte) 0b00111110;
        characterRegister[30][1] = (byte) 0b00000000; // U
        characterMap.put('U', 30);
        characterRegister[31][0] = (byte) 0b00110000;
        characterRegister[31][1] = (byte) 0b00100100; // V
        characterMap.put('V', 31);
        characterRegister[32][0] = (byte) 0b00110110;
        characterRegister[32][1] = (byte) 0b00101000; // W
        characterMap.put('W', 32);
        characterRegister[33][0] = (byte) 0b00000000;
        characterRegister[33][1] = (byte) 0b00101101; // X
        characterMap.put('X', 33);
        characterRegister[34][0] = (byte) 0b00000000;
        characterRegister[34][1] = (byte) 0b00010101; // Y
        characterMap.put('Y', 34);
        characterRegister[35][0] = (byte) 0b00001001;
        characterRegister[35][1] = (byte) 0b00100100; // Z
        characterMap.put('Z', 35);
        characterRegister[36][0] = (byte) 0b00000000;
        characterRegister[36][1] = (byte) 0b00000000; // space
        characterMap.put(' ', 36);

        // Configure
        writeBufferDelayed(OSC);
        writeBufferDelayed(BRIGHT);
        writeBufferDelayed(BLINK);
    }

    public enum Button {
        BUTTON_A, BUTTON_B
    }
}
