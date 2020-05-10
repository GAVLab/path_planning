clc; clear all; close all
remoteVeh = load('vehStates.mat');
sensors = load('sensors.mat');

%% Discretize the system

dt1 = 1/20;
dt2 = .5;

%define vehicle parameters (g35 vehicle parameters)
caf = 2*4.5837e+004; %corner stiffness front
car = 2*7.6394e+004; %corner stiffness rear

L = 2.8499;%length of wheel base
a = .48*L;%distance from cg to front axle
b =  .52*L;%distance from cg to rear axle

Iz = 2400; %yaww inertia
m = 1528.2; %vehicle mass

b11 = (a*caf / Iz);
b22 = caf / m;

B = [b11 b22 0 0 0 0 0 0]';

bWeights = [1 1 1 1 1 1 1 1];
Bw = bWeights.*eye(8);
qWeights = [1e-8 1e-2 1 1e-1 .0001 .0001 1e-10 1e-10];
Qc = qWeights.*eye(8);
Qd = Bw*Qc*Bw'.*dt1;

rWeights1 = [1e-3 1e-4 1e-5 1e-1 .0001 .0001];
rWeights2 = [1e-3 1e-3];

R1 = rWeights1.*eye(6);
R2 = rWeights2.*eye(2);

pInit = [3 3 1 10 10 10 15 15];

P = pInit.*eye(8);

x(:,1) = [remoteVeh.yawRate(1); remoteVeh.latVel(1); remoteVeh.Vx(1)+randn(); remoteVeh.heading(1); remoteVeh.xGlobal(1); remoteVeh.yGlobal(1); .8 ; 0];
xTruth(1) = remoteVeh.xGlobal(1);
yTruth(1) = remoteVeh.yGlobal(1);


%% do some actual filtering
i = 1;
t(i) = 0;
remoteCount = 1;
radarSample = 0;
drtkSample = 0;
counts = 20;

covarStates(:,i) = diag(P);

err(:,1) = zeros(6,1);
j = 1;
while t(i) < 50
    
if i == 1
    steerAngle = remoteVeh.steerAngle(remoteCount)*(pi/180);
end

stm = genStm(x(2,i),x(3,i),dt1);

%20Hz state update
xPriori(:,i) = stm*x(:,i) + dt1*B.*steerAngle;

pPriori = stm*P*stm' + Qd;

if counts == inv(dt1)

    gyro = sensors.gyro.scaleFactor*remoteVeh.yawRate(remoteCount) + sensors.gyro.bias + sensors.gyro.stdv*randn();
    Xdrtk = remoteVeh.xGlobal(remoteCount) + sensors.drtk.stdv*randn();
    Ydrtk = remoteVeh.yGlobal(remoteCount) + sensors.drtk.stdv*randn();
    gpsLatVel = remoteVeh.latVel(remoteCount) + sensors.gps.stdv*randn();
    gpsLongVel = remoteVeh.Vx(remoteCount) + sensors.gps.stdv*randn();
    headingMeas = remoteVeh.heading(remoteCount) + sensors.heading.stdv*randn();

    cMat2 = genCmat(x(4,i),x(7,i),dt2);
    kGain = pPriori*cMat2'*inv(cMat2*pPriori*cMat2' + R1);
   
    y = [gyro; gpsLatVel; gpsLongVel; headingMeas; Xdrtk; Ydrtk];

    x(:,i+1) = xPriori(:,i) + kGain*(y - cMat2*xPriori(:,i));

    err(:,j+1) = (y - cMat2*xPriori(:,i));

    xTruth(i+1) = remoteVeh.xGlobal(remoteCount);
    yTruth(i+1) = remoteVeh.yGlobal(remoteCount);

    P = (eye(8) - kGain*cMat2)*pPriori  ;
    
    steerAngle = remoteVeh.steerAngle(remoteCount);
    covarStates(:,i+1) = diag(P);
 
    counts = 1;
    j = j+1;
    
else

    gyro = sensors.gyro.scaleFactor*remoteVeh.yawRate(remoteCount) + sensors.gyro.bias + sensors.gyro.stdv*randn();
    gpsLatVel = remoteVeh.latVel(remoteCount) + sensors.gps.stdv*randn();
    gpsLongVel = remoteVeh.Vx(remoteCount) + sensors.gps.stdv*randn();
    headingMeas = remoteVeh.heading(remoteCount) + sensors.heading.stdv*randn();

    cMat1 = genCmat(x(4,i),x(7,i),dt1)  ;  
    
    kGain = pPriori*cMat1'*inv(cMat1*pPriori*cMat1' + R2);

    y = [gyro; headingMeas];

    x(:,i+1) = xPriori(:,i) + kGain*(y - cMat1*xPriori(:,i));

    xTruth(i+1) = remoteVeh.xGlobal(remoteCount);
    yTruth(i+1) = remoteVeh.yGlobal(remoteCount);

    j = j+1;
    P = (eye(8) - kGain*cMat1)*pPriori  ;
    steerAngle = remoteVeh.steerAngle(remoteCount);
    covarStates(:,i+1) = diag(P);

end


t(i+1) = t(i) + dt1;

counts = counts+1;
remoteCount = remoteCount + 5;

i = i+1;
end

%% plot stuff

figure(1)
plot(t,x(1,:),'LineWidth',2.25)
hold on
plot(remoteVeh.time,remoteVeh.yawRate.*(180/pi),'--','LineWidth',2.25)
%plot(remoteVeh.time,remoteVeh.yGlobal(1:length(remoteVeh.time)),'--','LineWidth',2.25)
legend('Estimated Lateral Path','True Lateral Path')

figure(2)
subplot(2,1,1)
plot(t,x(8,:),'LineWidth',2.25)
ylabel('Gyroscope Bias')
set(gca,'Fontsize',14)
subplot(2,1,2)
plot(t,x(7,:),'LineWidth',2.25)
ylabel('Gyroscope SF')
set(gca,'Fontsize',14)

figure(3)
subplot(2,1,1)
plot(t,x(2,:),'LineWidth',2.25)
hold on
plot(remoteVeh.time,remoteVeh.latVel,'LineWidth',2.25)
ylabel('lateral velocity')
legend('Estimate','Truth')
set(gca,'Fontsize',14)
subplot(2,1,2)
plot(t,x(3,:),'LineWidth',2.25)
hold on
plot(remoteVeh.time,remoteVeh.Vx,'LineWidth',2.25)
ylabel('longitudinal velocity')
xlabel('Time (s)')
set(gca,'Fontsize',14)

figure(4)
plot(t,x(4,:),'lineWidth',2.25)
hold on
plot(remoteVeh.time,remoteVeh.heading,'lineWidth',2.25)
ylabel('Heading')
legend('Estimated','Truth')

figure(5)
plot(t,x(5,:),'Linewidth',2.25)
hold on
plot(remoteVeh.time,remoteVeh.xGlobal(1:length(remoteVeh.time)),'LineWidth',2.25)
xlabel('Global X-position (m)')
ylabel('Time (m)')

figure(6)
plot(t,x(6,:),'Linewidth',2.25)
hold on
plot(remoteVeh.time,remoteVeh.yGlobal(1:length(remoteVeh.time)),'LineWidth',2.25)
ylabel('Global Y-position (m)')
xlabel('Time (m)')

figure(7)
plot(x(5,:),x(6,:),'Linewidth',2.25)
hold on
plot(remoteVeh.xGlobal,remoteVeh.yGlobal,'LineWidth',2.25)
legend('Estimated Path','Truth')
xlabel('X-Global (m)')
ylabel('Y-Global (m)')
set(gca,'FontSize',14)

%covariance stuff
figure(8)
subplot(2,1,1)
plot(t,covarStates(5,:),'LineWidth',2.25)
ylabel('x-pos covariance')
set(gca,'Fontsize',14)
axis([0 25 0 1e-3])
subplot(2,1,2)
plot(t,covarStates(6,:),'LineWidth',2.25)
ylabel('y-pos covariance')
xlabel('Time (s)')
set(gca,'Fontsize',14)
axis([0 25 0 .03])

%% Residuals
xPosErr = x(5,:) - xTruth;
yPosErr = x(6,:) - yTruth;
figure(9)
subplot(2,1,1)
plot(t,xPosErr,'LineWidth',2.25)
ylabel({'X-position';'Error (m)'})
set(gca,'FontSize',14)
subplot(2,1,2)
plot(t,yPosErr,'LineWidth',2.25)
ylabel({'Y-position';'Error (m)'})
xlabel('Time')


%% Functions
function stm = genStm(latVel,Vx,dt)

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
a11 = -c2 / (Iz*Vx);
a12 = -c1 / (Iz*Vx);
a21 = (-c1 / (m*Vx)) - Vx;
a22 = -c0 / (m*Vx);

A = [ a11 a12 0 0 0 0 0 0; ...
      a21 a22 0 0 0 0 0 0; ...
      0 0 0 0 0 0 0 0;...
      1 0 0 0 0 0 0 0;...
      0 0 1 -latVel 0 0 0 0;...
      0 1 0 Vx 0 0 0 0;...
      0 0 0 0 0 0 0 0;...
      0 0 0 0 0 0 0 0];
  
stm = eye(8) + A*dt; 


end

function cMat = genCmat(yawRate,a,dt)

if dt == .05
    cMat = [a 0 0 0 0 0 yawRate 1;...
            0 0 0 1 0 0 0 0];
end
if dt == .5
    cMat = [a 0 0 0 0 0 yawRate 1;...
            0 1 0 0 0 0 0 0;...
            0 0 1 0 0 0 0 0;...
            0 0 0 1 0 0 0 0;...
            0 0 0 0 1 0 0 0;...
            0 0 0 0 0 1 0 0];
end

end





