/**
 * Copyright (c) 2019 Team 3555
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.aluminati3555.lib.net;

import java.io.ByteArrayInputStream;
import java.io.DataInputStream;
import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.SocketException;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

/**
 * This class listens for UDP packets and calls an abstract method to update PID
 * values
 * 
 * @author Caleb Heydon
 */
public abstract class AluminatiTunable extends Thread {
    private static final int PACKET_LENGTH = 24;

    /**
     * Sends updated tuning data to a system
     * 
     * @param data
     */
    protected abstract void update(TuningData data);

    private int port;
    private DatagramSocket socket;
    private DatagramPacket packet;

    /**
     * Returns a useful string
     */
    @Override
    public String toString() {
        return "[Tunable] port: " + port;
    }

    /**
     * Called when the thread is started
     */
    @Override
    public void run() {
        while (!DriverStation.getInstance().isFMSAttached()) {
            try {
                socket.receive(packet);

                ByteArrayInputStream byteInput = new ByteArrayInputStream(packet.getData());
                DataInputStream input = new DataInputStream(byteInput);

                double kP = input.readDouble();
                double kI = input.readDouble();
                double kD = input.readDouble();
                input.close();

                update(new TuningData(kP, kI, kD));
            } catch (IOException e) {
                Timer.delay(1);
                continue;
            }
        }

        socket.close();
    }

    public AluminatiTunable(int port) {
        System.out.println("Starting UDP listener for Tunable on port " + port + "...");

        this.port = port;
        try {
            this.socket = new DatagramSocket(port);
        } catch (SocketException e) {
            DriverStation.reportError("Unable to start UDP listener for Tunable on port " + port, false);
            return;
        }
        this.packet = new DatagramPacket(new byte[PACKET_LENGTH], PACKET_LENGTH);

        super.setName("Tunable-" + port);
        super.setPriority(Thread.MIN_PRIORITY);
        super.start();
    }

    public class TuningData {
        public double kP;
        public double kI;
        public double kD;

        public TuningData(double kP, double kI, double kD) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
        }
    }
}
