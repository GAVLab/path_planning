clc;clear all;close all


runTime = 4*pi*4;%800;
samples= 100*runTime;
%set time of the sim
t = linspace(0,runTime,samples);
dt = runTime/samples;
%set simulation parameters

steerAngle = .5.*sin(.25*t);
%steerAngle = .125.*ones(1,length(t));

figure(1)
plot(t,steerAngle,'LineWidth',2.25)
ylabel('Steer Angle')
xlabel('Time')
%assume cruise control keeps a constant velocity
Vx = 13.5 + .25.*sin(.25*t);
rDot(1) = 0;
latVel(1) = 0;
yawRate(1) = 0;
heading(1) = 0;

%% Simulate Data
for i = 1:length(t)-1
    
    [rDot(i+1),vyDot(i+1)] = bicycleModel(steerAngle(i),Vx(i),latVel(i),sqrt(Vx(i)^2 + latVel(i)^2),yawRate(i));
    
    yawRate(i+1) = yawRate(i) + rDot(i+1)*dt;
    
    heading(i+1) = heading(i) + yawRate(i+1)*dt;
    
    latVel(i+1) = latVel(i) + vyDot(i+1)*dt;
    
end

rDot = rDot.*(pi/180);
yawRate = yawRate.*(pi/180);
heading= heading.*(pi/180);

for i = 1:length(latVel)
    sideSlip(i) = atan(latVel(i)/Vx(i));
end

course = heading - sideSlip;

figure(2)
subplot(3,1,1)
plot(t,yawRate,'LineWidth',2.25)
ylabel({'Yaw Rate';'(deg/s)'})
set(gca,'FontSize',14)
subplot(3,1,2)
plot(t,heading,'LineWidth',2.25)
ylabel({'heading' ; '(radians) (deg/s)'})
set(gca,'FontSize',14)
subplot(3,1,3)
plot(t,latVel,'LineWidth',2.25)
ylabel('Lateral Velocity')
xlabel('Time (s)')
set(gca,'FontSize',14)

figure(3)
plot(t,sideSlip*(180/pi),'LineWidth',2.25)
ylabel('Sideslip Angle')
xlabel('time')
set(gca,'Fontsize',14)


%% Get Vehicle Position
x_2dot(1) = 0;
x_dot(1) = 0;
x(1) = 0;
y_2dot(1) = 0;
y_dot(1) = 0;
y(1) = 0;
x(1) = 30;

for i = 1:length(yawRate)
    
    
vyGlobal(i) = Vx(i)*sin(heading(i)) + latVel(i)*cos(heading(i));

y(i+1) = y(i) + vyGlobal(i)*dt;

vxGlobal(i) = Vx(i)*cos(heading(i)) - latVel(i)*sin(heading(i));
x(i+1) = x(i) + vxGlobal(i)*dt;

end

figure(4)
subplot(2,1,1)
plot(t,vxGlobal,'lineWidth',2.25)
ylabel('VX-Global')
set(gca,'FontSize',14)
subplot(2,1,2)
plot(t,vyGlobal,'lineWidth',2.25)
ylabel('VY-Global')
xlabel('Time (s)')
set(gca,'FontSize',14)

figure(5)
plot(x,y,'LineWidth',2.25)
xlabel('Global Y-pos (m)')
ylabel('Global X-pos (m)')
set(gca,'FontSize',14)

vehStates.steerAngle = steerAngle;
vehStates.xGlobal = x;
vehStates.yGlobal = y;
vehStates.heading = heading;
vehStates.course = course
vehStates.yawAccel = rDot;
vehStates.yawRate = yawRate;
vehStates.latVel = latVel;
vehStates.Vx = Vx;
vehStates.sideSlep = sideSlip;
vehStates.dt = .0126;
vehStates.vxGlobal = vxGlobal;
vehStates.vyGlobal = vyGlobal;
vehStates.time = t;

save('vehStates.mat','-struct','vehStates')









