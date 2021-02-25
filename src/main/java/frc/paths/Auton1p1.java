package frc.paths;

import com.team319.trajectory.Path;

public class Auton1p1 extends Path {
   // dt,x,y,left.pos,left.vel,left.acc,left.jerk,center.pos,center.vel,center.acc,center.jerk,right.pos,right.vel,right.acc,right.jerk,heading
	private static final double[][] points = {
				{0.0200,40.9660,-5.6900,0.0040,0.2000,10.0000,0.0000,0.0040,0.2000,10.0000,0.0000,0.0040,0.2000,10.0000,0.0000,3.1416},
				{0.0200,40.9580,-5.6900,0.0120,0.4000,10.0000,0.0000,0.0120,0.4000,10.0000,0.0000,0.0120,0.4000,10.0000,0.0000,3.1416},
				{0.0200,40.9460,-5.6900,0.0240,0.6000,10.0000,-0.0000,0.0240,0.6000,10.0000,0.0000,0.0240,0.6000,10.0000,-0.0000,3.1416},
				{0.0200,40.9300,-5.6900,0.0400,0.8000,10.0000,0.0000,0.0400,0.8000,10.0000,0.0000,0.0400,0.8000,10.0000,0.0000,3.1416},
				{0.0200,40.9100,-5.6900,0.0600,1.0000,10.0000,0.0000,0.0600,1.0000,10.0000,0.0000,0.0600,1.0000,10.0000,0.0000,3.1416},
				{0.0200,40.8860,-5.6900,0.0840,1.2000,10.0000,-0.0000,0.0840,1.2000,10.0000,0.0000,0.0840,1.2000,10.0000,-0.0000,3.1416},
				{0.0200,40.8580,-5.6900,0.1120,1.4000,10.0000,0.0000,0.1120,1.4000,10.0000,0.0000,0.1120,1.4000,10.0000,0.0000,3.1416},
				{0.0200,40.8260,-5.6900,0.1440,1.6000,10.0000,0.0000,0.1440,1.6000,10.0000,0.0000,0.1440,1.6000,10.0000,0.0000,3.1416},
				{0.0200,40.7900,-5.6900,0.1800,1.8000,10.0000,-0.0000,0.1800,1.8000,10.0000,0.0000,0.1800,1.8000,10.0000,-0.0000,3.1416},
				{0.0200,40.7500,-5.6900,0.2200,2.0000,10.0000,0.0000,0.2200,2.0000,10.0000,0.0000,0.2200,2.0000,10.0000,0.0000,3.1416},
				{0.0200,40.7060,-5.6900,0.2640,2.2000,10.0000,0.0000,0.2640,2.2000,10.0000,0.0000,0.2640,2.2000,10.0000,0.0000,3.1416},
				{0.0200,40.6580,-5.6900,0.3120,2.4000,10.0000,-0.0000,0.3120,2.4000,10.0000,0.0000,0.3120,2.4000,10.0000,-0.0000,3.1416},
				{0.0200,40.6060,-5.6900,0.3640,2.6000,10.0000,-0.0000,0.3640,2.6000,10.0000,0.0000,0.3640,2.6000,10.0000,-0.0000,3.1416},
				{0.0200,40.5500,-5.6900,0.4200,2.8000,10.0000,0.0000,0.4200,2.8000,10.0000,0.0000,0.4200,2.8000,10.0000,0.0000,3.1416},
				{0.0200,40.4900,-5.6900,0.4800,3.0000,10.0000,0.0000,0.4800,3.0000,10.0000,0.0000,0.4800,3.0000,10.0000,0.0000,3.1416},
				{0.0200,40.4260,-5.6900,0.5440,3.2000,10.0000,0.0000,0.5440,3.2000,10.0000,0.0000,0.5440,3.2000,10.0000,0.0000,3.1416},
				{0.0200,40.3580,-5.6900,0.6120,3.4000,10.0000,0.0000,0.6120,3.4000,10.0000,0.0000,0.6120,3.4000,10.0000,0.0000,3.1416},
				{0.0200,40.2860,-5.6900,0.6840,3.6000,10.0000,0.0000,0.6840,3.6000,10.0000,0.0000,0.6840,3.6000,10.0000,0.0000,3.1416},
				{0.0200,40.2100,-5.6900,0.7600,3.8000,10.0000,-0.0000,0.7600,3.8000,10.0000,0.0000,0.7600,3.8000,10.0000,-0.0000,3.1416},
				{0.0200,40.1300,-5.6900,0.8400,4.0000,10.0000,-0.0000,0.8400,4.0000,10.0000,0.0000,0.8400,4.0000,10.0000,-0.0000,3.1416},
				{0.0200,40.0460,-5.6900,0.9240,4.2000,10.0000,-0.0000,0.9240,4.2000,10.0000,0.0000,0.9240,4.2000,10.0000,-0.0000,3.1416},
				{0.0200,39.9580,-5.6900,1.0120,4.4000,10.0000,0.0000,1.0120,4.4000,10.0000,0.0000,1.0120,4.4000,10.0000,0.0000,3.1416},
				{0.0200,39.8660,-5.6900,1.1040,4.6000,10.0000,-0.0000,1.1040,4.6000,10.0000,0.0000,1.1040,4.6000,10.0000,-0.0000,3.1416},
				{0.0200,39.7700,-5.6900,1.2000,4.8000,10.0000,0.0000,1.2000,4.8000,10.0000,0.0000,1.2000,4.8000,10.0000,0.0000,3.1416},
				{0.0200,39.6700,-5.6900,1.3000,5.0000,10.0000,-0.0000,1.3000,5.0000,10.0000,0.0000,1.3000,5.0000,10.0000,-0.0000,3.1416},
				{0.0200,39.5660,-5.6900,1.4040,5.2000,10.0000,-0.0000,1.4040,5.2000,10.0000,0.0000,1.4040,5.2000,10.0000,-0.0000,3.1416},
				{0.0200,39.4580,-5.6900,1.5120,5.4000,10.0000,0.0000,1.5120,5.4000,10.0000,0.0000,1.5120,5.4000,10.0000,0.0000,3.1416},
				{0.0200,39.3460,-5.6900,1.6240,5.6000,10.0000,0.0000,1.6240,5.6000,10.0000,0.0000,1.6240,5.6000,10.0000,0.0000,3.1416},
				{0.0200,39.2300,-5.6900,1.7400,5.8000,10.0000,0.0000,1.7400,5.8000,10.0000,0.0000,1.7400,5.8000,10.0000,0.0000,3.1416},
				{0.0200,39.1100,-5.6900,1.8600,6.0000,10.0000,-0.0000,1.8600,6.0000,10.0000,0.0000,1.8600,6.0000,10.0000,-0.0000,3.1416},
				{0.0200,38.9860,-5.6900,1.9840,6.2000,10.0000,0.0000,1.9840,6.2000,10.0000,0.0000,1.9840,6.2000,10.0000,0.0000,3.1416},
				{0.0200,38.8580,-5.6900,2.1120,6.4000,10.0000,-0.0000,2.1120,6.4000,10.0000,0.0000,2.1120,6.4000,10.0000,-0.0000,3.1416},
				{0.0200,38.7260,-5.6900,2.2440,6.6000,10.0000,0.0000,2.2440,6.6000,10.0000,0.0000,2.2440,6.6000,10.0000,0.0000,3.1416},
				{0.0200,38.5900,-5.6900,2.3800,6.8000,10.0000,-0.0000,2.3800,6.8000,10.0000,0.0000,2.3800,6.8000,10.0000,-0.0000,3.1416},
				{0.0200,38.4500,-5.6900,2.5200,7.0000,10.0000,-0.0000,2.5200,7.0000,10.0000,0.0000,2.5200,7.0000,10.0000,-0.0000,3.1416},
				{0.0200,38.3060,-5.6900,2.6640,7.2000,10.0000,0.0000,2.6640,7.2000,10.0000,0.0000,2.6640,7.2000,10.0000,0.0000,3.1416},
				{0.0200,38.1580,-5.6900,2.8120,7.4000,10.0000,0.0000,2.8120,7.4000,10.0000,0.0000,2.8120,7.4000,10.0000,0.0000,3.1416},
				{0.0200,38.0060,-5.6900,2.9640,7.6000,10.0000,-0.0000,2.9640,7.6000,10.0000,0.0000,2.9640,7.6000,10.0000,-0.0000,3.1416},
				{0.0200,37.8500,-5.6900,3.1200,7.8000,10.0000,0.0000,3.1200,7.8000,10.0000,0.0000,3.1200,7.8000,10.0000,0.0000,3.1416},
				{0.0200,37.6900,-5.6900,3.2800,8.0000,10.0000,0.0000,3.2800,8.0000,10.0000,0.0000,3.2800,8.0000,10.0000,0.0000,3.1416},
				{0.0200,37.5260,-5.6900,3.4440,8.2000,10.0000,0.0000,3.4440,8.2000,10.0000,0.0000,3.4440,8.2000,10.0000,0.0000,3.1416},
				{0.0200,37.3580,-5.6900,3.6120,8.4000,10.0000,0.0000,3.6120,8.4000,10.0000,0.0000,3.6120,8.4000,10.0000,0.0000,3.1416},
				{0.0200,37.1860,-5.6900,3.7840,8.6000,10.0000,-0.0000,3.7840,8.6000,10.0000,0.0000,3.7840,8.6000,10.0000,-0.0000,3.1416},
				{0.0200,37.0100,-5.6900,3.9600,8.8000,10.0000,0.0000,3.9600,8.8000,10.0000,0.0000,3.9600,8.8000,10.0000,0.0000,3.1416},
				{0.0200,36.8300,-5.6900,4.1400,9.0000,10.0000,-0.0000,4.1400,9.0000,10.0000,0.0000,4.1400,9.0000,10.0000,-0.0000,3.1416},
				{0.0200,36.6460,-5.6900,4.3240,9.2000,10.0000,0.0000,4.3240,9.2000,10.0000,0.0000,4.3240,9.2000,10.0000,0.0000,3.1416},
				{0.0200,36.4580,-5.6900,4.5120,9.4000,10.0000,-0.0000,4.5120,9.4000,10.0000,0.0000,4.5120,9.4000,10.0000,-0.0000,3.1416},
				{0.0200,36.2660,-5.6900,4.7040,9.6000,10.0000,0.0000,4.7040,9.6000,10.0000,0.0000,4.7040,9.6000,10.0000,0.0000,3.1416},
				{0.0200,36.0700,-5.6900,4.9000,9.8000,10.0000,0.0000,4.9000,9.8000,10.0000,0.0000,4.9000,9.8000,10.0000,0.0000,3.1416},
				{0.0200,35.8700,-5.6900,5.1000,10.0000,10.0000,0.0000,5.1000,10.0000,10.0000,0.0000,5.1000,10.0000,10.0000,0.0000,3.1416},
				{0.0200,35.6660,-5.6900,5.3040,10.2000,10.0000,0.0000,5.3040,10.2000,10.0000,0.0000,5.3040,10.2000,10.0000,0.0000,3.1416},
				{0.0200,35.4660,-5.6900,5.5040,10.0000,-10.0000,-1000.0000,5.5040,10.0000,-10.0000,0.0000,5.5040,10.0000,-10.0000,-1000.0000,3.1416},
				{0.0200,35.2700,-5.6900,5.7000,9.8000,-10.0000,-0.0000,5.7000,9.8000,-10.0000,0.0000,5.7000,9.8000,-10.0000,-0.0000,3.1416},
				{0.0200,35.0780,-5.6900,5.8920,9.6000,-10.0000,0.0000,5.8920,9.6000,-10.0000,0.0000,5.8920,9.6000,-10.0000,0.0000,3.1416},
				{0.0200,34.8900,-5.6900,6.0800,9.4000,-10.0000,0.0000,6.0800,9.4000,-10.0000,0.0000,6.0800,9.4000,-10.0000,0.0000,3.1416},
				{0.0200,34.7060,-5.6900,6.2640,9.2000,-10.0000,0.0000,6.2640,9.2000,-10.0000,0.0000,6.2640,9.2000,-10.0000,0.0000,3.1416},
				{0.0200,34.5260,-5.6900,6.4440,9.0000,-10.0000,0.0000,6.4440,9.0000,-10.0000,0.0000,6.4440,9.0000,-10.0000,0.0000,3.1416},
				{0.0200,34.3500,-5.6900,6.6200,8.8000,-10.0000,0.0000,6.6200,8.8000,-10.0000,0.0000,6.6200,8.8000,-10.0000,0.0000,3.1416},
				{0.0200,34.1780,-5.6900,6.7920,8.6000,-10.0000,-0.0000,6.7920,8.6000,-10.0000,0.0000,6.7920,8.6000,-10.0000,-0.0000,3.1416},
				{0.0200,34.0100,-5.6900,6.9600,8.4000,-10.0000,0.0000,6.9600,8.4000,-10.0000,0.0000,6.9600,8.4000,-10.0000,0.0000,3.1416},
				{0.0200,33.8460,-5.6900,7.1240,8.2000,-10.0000,-0.0000,7.1240,8.2000,-10.0000,0.0000,7.1240,8.2000,-10.0000,-0.0000,3.1416},
				{0.0200,33.6860,-5.6900,7.2840,8.0000,-10.0000,0.0000,7.2840,8.0000,-10.0000,0.0000,7.2840,8.0000,-10.0000,0.0000,3.1416},
				{0.0200,33.5300,-5.6900,7.4400,7.8000,-10.0000,0.0000,7.4400,7.8000,-10.0000,0.0000,7.4400,7.8000,-10.0000,0.0000,3.1416},
				{0.0200,33.3780,-5.6900,7.5920,7.6000,-10.0000,0.0000,7.5920,7.6000,-10.0000,0.0000,7.5920,7.6000,-10.0000,0.0000,3.1416},
				{0.0200,33.2300,-5.6900,7.7400,7.4000,-10.0000,0.0000,7.7400,7.4000,-10.0000,0.0000,7.7400,7.4000,-10.0000,0.0000,3.1416},
				{0.0200,33.0860,-5.6900,7.8840,7.2000,-10.0000,0.0000,7.8840,7.2000,-10.0000,0.0000,7.8840,7.2000,-10.0000,0.0000,3.1416},
				{0.0200,32.9460,-5.6900,8.0240,7.0000,-10.0000,-0.0000,8.0240,7.0000,-10.0000,0.0000,8.0240,7.0000,-10.0000,-0.0000,3.1416},
				{0.0200,32.8100,-5.6900,8.1600,6.8000,-10.0000,-0.0000,8.1600,6.8000,-10.0000,0.0000,8.1600,6.8000,-10.0000,-0.0000,3.1416},
				{0.0200,32.6780,-5.6900,8.2920,6.6000,-10.0000,0.0000,8.2920,6.6000,-10.0000,0.0000,8.2920,6.6000,-10.0000,0.0000,3.1416},
				{0.0200,32.5500,-5.6900,8.4200,6.4000,-10.0000,-0.0000,8.4200,6.4000,-10.0000,0.0000,8.4200,6.4000,-10.0000,-0.0000,3.1416},
				{0.0200,32.4260,-5.6900,8.5440,6.2000,-10.0000,0.0000,8.5440,6.2000,-10.0000,0.0000,8.5440,6.2000,-10.0000,0.0000,3.1416},
				{0.0200,32.3060,-5.6900,8.6640,6.0000,-10.0000,0.0000,8.6640,6.0000,-10.0000,0.0000,8.6640,6.0000,-10.0000,0.0000,3.1416},
				{0.0200,32.1900,-5.6900,8.7800,5.8000,-10.0000,0.0000,8.7800,5.8000,-10.0000,0.0000,8.7800,5.8000,-10.0000,0.0000,3.1416},
				{0.0200,32.0780,-5.6900,8.8920,5.6000,-10.0000,0.0000,8.8920,5.6000,-10.0000,0.0000,8.8920,5.6000,-10.0000,0.0000,3.1416},
				{0.0200,31.9700,-5.6900,9.0000,5.4000,-10.0000,-0.0000,9.0000,5.4000,-10.0000,0.0000,9.0000,5.4000,-10.0000,-0.0000,3.1416},
				{0.0200,31.8660,-5.6900,9.1040,5.2000,-10.0000,0.0000,9.1040,5.2000,-10.0000,0.0000,9.1040,5.2000,-10.0000,0.0000,3.1416},
				{0.0200,31.7660,-5.6900,9.2040,5.0000,-10.0000,0.0000,9.2040,5.0000,-10.0000,0.0000,9.2040,5.0000,-10.0000,0.0000,3.1416},
				{0.0200,31.6700,-5.6900,9.3000,4.8000,-10.0000,-0.0000,9.3000,4.8000,-10.0000,0.0000,9.3000,4.8000,-10.0000,-0.0000,3.1416},
				{0.0200,31.5780,-5.6900,9.3920,4.6000,-10.0000,0.0000,9.3920,4.6000,-10.0000,0.0000,9.3920,4.6000,-10.0000,0.0000,3.1416},
				{0.0200,31.4900,-5.6900,9.4800,4.4000,-10.0000,0.0000,9.4800,4.4000,-10.0000,0.0000,9.4800,4.4000,-10.0000,0.0000,3.1416},
				{0.0200,31.4060,-5.6900,9.5640,4.2000,-10.0000,0.0000,9.5640,4.2000,-10.0000,0.0000,9.5640,4.2000,-10.0000,0.0000,3.1416},
				{0.0200,31.3260,-5.6900,9.6440,4.0000,-10.0000,-0.0000,9.6440,4.0000,-10.0000,0.0000,9.6440,4.0000,-10.0000,-0.0000,3.1416},
				{0.0200,31.2500,-5.6900,9.7200,3.8000,-10.0000,0.0000,9.7200,3.8000,-10.0000,0.0000,9.7200,3.8000,-10.0000,0.0000,3.1416},
				{0.0200,31.1780,-5.6900,9.7920,3.6000,-10.0000,0.0000,9.7920,3.6000,-10.0000,0.0000,9.7920,3.6000,-10.0000,0.0000,3.1416},
				{0.0200,31.1100,-5.6900,9.8600,3.4000,-10.0000,-0.0000,9.8600,3.4000,-10.0000,0.0000,9.8600,3.4000,-10.0000,-0.0000,3.1416},
				{0.0200,31.0460,-5.6900,9.9240,3.2000,-10.0000,0.0000,9.9240,3.2000,-10.0000,0.0000,9.9240,3.2000,-10.0000,0.0000,3.1416},
				{0.0200,30.9860,-5.6900,9.9840,3.0000,-10.0000,-0.0000,9.9840,3.0000,-10.0000,0.0000,9.9840,3.0000,-10.0000,-0.0000,3.1416},
				{0.0200,30.9300,-5.6900,10.0400,2.8000,-10.0000,0.0000,10.0400,2.8000,-10.0000,0.0000,10.0400,2.8000,-10.0000,0.0000,3.1416},
				{0.0200,30.8780,-5.6900,10.0920,2.6000,-10.0000,-0.0000,10.0920,2.6000,-10.0000,0.0000,10.0920,2.6000,-10.0000,-0.0000,3.1416},
				{0.0200,30.8300,-5.6900,10.1400,2.4000,-10.0000,-0.0000,10.1400,2.4000,-10.0000,0.0000,10.1400,2.4000,-10.0000,-0.0000,3.1416},
				{0.0200,30.8300,-5.6900,10.1400,0.0000,-120.0000,-5500.0000,10.1400,2.2000,-10.0000,0.0000,10.1400,0.0000,-120.0000,-5500.0000,3.1416},
				{0.0200,30.8300,-5.6900,10.1400,0.0000,0.0000,6000.0000,10.1400,2.0000,-10.0000,0.0000,10.1400,0.0000,0.0000,6000.0000,3.1416},
				{0.0200,30.8300,-5.6900,10.1400,0.0000,0.0000,0.0000,10.1400,1.8000,-10.0000,0.0000,10.1400,0.0000,0.0000,0.0000,3.1416},
				{0.0200,30.8300,-5.6900,10.1400,0.0000,0.0000,0.0000,10.1400,1.6000,-10.0000,0.0000,10.1400,0.0000,0.0000,0.0000,3.1416},
				{0.0200,30.8300,-5.6900,10.1400,0.0000,0.0000,0.0000,10.1400,1.4000,-10.0000,0.0000,10.1400,0.0000,0.0000,0.0000,3.1416},
				{0.0200,30.8300,-5.6900,10.1400,0.0000,0.0000,0.0000,10.1400,1.2000,-10.0000,0.0000,10.1400,0.0000,0.0000,0.0000,3.1416},
				{0.0200,30.8300,-5.6900,10.1400,0.0000,0.0000,0.0000,10.1400,1.0000,-10.0000,0.0000,10.1400,0.0000,0.0000,0.0000,3.1416},
				{0.0200,30.8300,-5.6900,10.1400,0.0000,0.0000,0.0000,10.1400,0.8000,-10.0000,0.0000,10.1400,0.0000,0.0000,0.0000,3.1416},
				{0.0200,30.8300,-5.6900,10.1400,0.0000,0.0000,0.0000,10.1400,0.6000,-10.0000,0.0000,10.1400,0.0000,0.0000,0.0000,3.1416},
				{0.0200,30.8300,-5.6900,10.1400,0.0000,0.0000,0.0000,10.1400,0.4000,-10.0000,0.0000,10.1400,0.0000,0.0000,0.0000,3.1416},
				{0.0200,30.8300,-5.6900,10.1400,0.0000,0.0000,0.0000,10.1400,0.2000,-10.0000,0.0000,10.1400,0.0000,0.0000,0.0000,3.1416},
				{0.0200,30.8300,-5.6900,10.1400,0.0000,0.0000,0.0000,10.1400,-0.0000,-10.0000,0.0000,10.1400,0.0000,0.0000,0.0000,3.1416},

	    };

	@Override
	public double[][] getPath() {
	    return points;
	}
}