function [yawAccel , latAccel] = bicycleModel(steerAngle,longVel,latVel,velMag,yawRate)
%BICYCLEMODEL Summary of this function goes here
%   Detailed explanation goes here

%define vehicle parameters (g35 vehicle parameters)
caf = 2*4.5837e+004; %corner stiffness front
car = 2*7.6394e+004; %corner stiffness rear

L = 2.8499;%length of wheel base
a = .48*L;%distance from cg to front axle
b =  .52*L;%distance from cg to rear axle

Iz = 2400; %yaww inertia
m = 1528.2; %vehicle mass

%handling characteristics
c0 = caf + car;
c1 = a*caf - b*car;
c2 = a^2*caf + b^2*car;

%components of the A matrix
a11 = -c2 / (Iz*longVel);
a12 = -c1 / (Iz*longVel);
a21 = (-c1 / (m*longVel)) - velMag;
a22 = -c0 / (m*longVel);

b11 = (a*caf / Iz);
b22 = caf / m;

%components of the B matric

states = [a11 , a12 ; a21 a22  ] * [yawRate ; latVel] + [b11 ; b22]*steerAngle;

yawAccel = states(1);

latAccel = states(2);

end

