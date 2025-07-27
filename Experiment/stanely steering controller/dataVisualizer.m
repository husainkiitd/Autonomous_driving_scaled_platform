
%%%%%%%%%%%%%%%%% race track %%%%%%%%%%%%%%%
R = 2;
D = 3;

x1 = 0;
y1 = 0;

th1 = 0:1:179;

for i = 1:180
    x2(i) = D/2 + R*sind(th1(i));
    y2(i) = R - R*cosd(th1(i));
end

for i = 1:180
    x3(i) = -D/2 - R*sind(th1(i));
    y3(i) = R + R*cosd(th1(i));

end
x4 = -0.1;
y4 = 0;

x = [x1 x2 x3 x4];
y = [y1 y2 y3 y4];

refPose =[x;y]'; 
xRef = refPose(:,1);
yRef = refPose(:,2);

% calculate distance vector
distancematrix = squareform(pdist(refPose));
distancesteps = zeros(length(refPose)-1,1);
for i = 2:length(refPose)
    distancesteps(i-1,1) = distancematrix(i,i-1);
end
totalDistance = sum(distancesteps); % Total distance travelled
distbp = cumsum([0; distancesteps]); % Distance for each waypoint
gradbp = linspace(0,totalDistance,1000); % Linearize distance

% linearize X and Y vectors based on distance
xRef2 = interp1(distbp,xRef,gradbp);
yRef2 = interp1(distbp,yRef,gradbp);
yRef2s = smooth(gradbp,yRef2); % smooth waypoints
xRef2s = smooth(gradbp,xRef2); % smooth waypoints
xRef = xRef2s;
yRef = yRef2s;


data = load("stnl10march_2.75.mat");

time =data.out.pose.time(:,1);
xx =data.out.pose.signals.values(:,1);
yy =data.out.pose.signals.values(:,2);
theta =data.out.pose.signals.values(:,3);
steering  =data.out.steering.signals.values(:,1);
yawrate  =data.out.IMU.signals.values(:,3);

mind =data.out.min_d.signals.values(:,1);

theta_e =data.out.theta_e.signals.values(:,1);


%%%% plot trajectory
figure(1)
plot(xx,yy,xRef,yRef,"LineWidth",1)
xlabel("X-axis in meter")
ylabel("Y-axis in meter")
title("Tracking")

%%%% plot steering angle
figure(2)
plot(time,steering,"LineWidth",1)
xlabel("Time in sec.")
ylabel("Steering angle in degree")
title("Time v/s steering angle")

%%%% plot states
figure(3)
plot(time,mind,"LineWidth",1)
xlabel("Time in sec.")
ylabel("Cross Track Error")
title("Time v/s Cross Track Error")


%%%% plot yaw rate
figure(4)
plot(time,yawrate,"LineWidth",1)
xlabel("Time in sec.")
ylabel("Yaw Rate in radian per sec.")
title("Time v/s Yaw Rate")
