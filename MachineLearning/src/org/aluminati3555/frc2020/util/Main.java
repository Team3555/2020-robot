package org.aluminati3555.frc2020.util;

import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

public class Main {
	private static final int[] GREEN = { 0, 255, 0 };

	public static void main(String[] args) throws IOException {
		ShooterUtil.create("C:\\Users\\C\\Desktop\\training.txt");
		ShooterUtil.save("C:\\Users\\C\\Desktop\\shooter.ml");

		BufferedImage image = new BufferedImage(1000, 100, BufferedImage.TYPE_INT_RGB);
		for (double i = 0; i <= 100; i += 0.01) {
			double rpm = ShooterUtil.calculateRPM(i);
			image.getRaster().setPixel((int) (i * 10), 99 - (int) (rpm / 100), GREEN);
		}

		ImageIO.write(image, "PNG", new File("C:\\Users\\C\\Desktop\\output.png"));
	}
}
