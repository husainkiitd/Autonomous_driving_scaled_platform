data = load("LQR7march25_1_30_0.5_10.mat");

time =data.out.pose.time(:,1);
xx =data.out.pose.signals.values(:,1);
yy =data.out.pose.signals.values(:,2);
theta =data.out.pose.signals.values(:,3);
steering  =data.out.steering.signals.values(:,1);
yawrate  =data.out.IMU.signals.values(:,3);

e1 =data.out.e1.signals.values(:,1);
e1dot =data.out.e1_dot.signals.values(:,1);
e2 =data.out.e2.signals.values(:,1);
e2dot =data.out.e2_dot.signals.values(:,1);

%%%% plot trajectory
figure(1)
plot(xx,yy,"LineWidth",1)
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
subplot(4, 1, 1);
plot(time,e1,"LineWidth",1)
xlabel("Time in sec.")
ylabel("e1")
title("Time v/s e1")

subplot(4, 1, 2);
plot(time,e1dot,"LineWidth",1)
xlabel("Time in sec.")
ylabel("e1dot")
title("Time v/s e1dot")

subplot(4, 1, 3);
plot(time,e2,"LineWidth",1)
xlabel("Time in sec.")
ylabel("e2")
title("Time v/s e2")

subplot(4, 1, 4);
plot(time,e2dot,"LineWidth",1)
xlabel("Time in sec.")
ylabel("e2dot")
title("Time v/s e2dot")
%%%% plot steering angle
figure(4)
plot(time,yawrate,"LineWidth",1)
xlabel("Time in sec.")
ylabel("Yaw Rate in radian per sec.")
title("Time v/s Yaw Rate")
