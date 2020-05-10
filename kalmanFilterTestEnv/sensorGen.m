sensors.drtk.mean = 0;
sensors.drtk.stdv = .04;

sensors.gyro.stdv = .2;
sensors.gyro.bias = .25;
sensors.gyro.scaleFactor = 2;

sensors.accel.scaleFactor = 1;
sensors.accel.bias = 2.7e-4;
sensors.accel.stdv = .35;

sensors.gpsVel.stdv = .03;
sensors.heading.stdv = .08;

%for now we'll define the steer angle and velocity emasurements to be good.


save('sensors.mat','-struct','sensors')




