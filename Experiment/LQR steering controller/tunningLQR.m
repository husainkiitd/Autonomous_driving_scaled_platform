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





data1 = load("LQR7march10_1_10_0.5_10.mat");
data2 = load("LQR7march25_1_20_0.5_10.mat");
data3 = load("LQR7march25_1_30_0.5_10.mat");
data4 = load("LQR7march30_1_30_0.5_10.mat");

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

time1 =data1.out.pose.time(:,1);
xx1 =data1.out.pose.signals.values(:,1);
yy1 =data1.out.pose.signals.values(:,2);
theta1 =data1.out.pose.signals.values(:,3);
mind1 =data1.out.e1.signals.values(:,1);
mind1(end) = NaN;
c1 = mind1;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
time2 =data2.out.pose.time(:,1);
xx2 =data2.out.pose.signals.values(:,1);
yy2 =data2.out.pose.signals.values(:,2);
theta2 =data2.out.pose.signals.values(:,3);
mind2 =data2.out.e1.signals.values(:,1);
mind2(end) = NaN;
c2 = mind2;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
time3 =data3.out.pose.time(:,1);
xx3 =data3.out.pose.signals.values(:,1);
yy3 =data3.out.pose.signals.values(:,2);
theta3 =data3.out.pose.signals.values(:,3);
mind3 =data3.out.e1.signals.values(:,1);
mind3(end) = NaN;
c3 = mind3;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
time4 =data4.out.pose.time(:,1);
xx4 =data4.out.pose.signals.values(:,1);
yy4 =data4.out.pose.signals.values(:,2);
theta4 =data4.out.pose.signals.values(:,3);
mind4 =data4.out.e1.signals.values(:,1);
mind4(end) = NaN;
c4 = mind4;
%%%%%%%%%%%%%%%%%
rms1 = rms(mind1(~isnan(mind1)));
rms2 = rms(mind2(~isnan(mind2)));
rms3 = rms(mind3(~isnan(mind3)));
rms4 = rms(mind4(~isnan(mind4)));

RMS = [rms1, rms2, rms3, rms4]
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% %%%% plot trajectory
% figure(1)
% plot(xx3,yy3,xRef,yRef,"LineWidth",2)
% xlabel("X-axis (m)",'interpreter','latex')
% ylabel("Y-axis (m)",'interpreter','latex')
% legend('Actual','Reference','interpreter','latex')
% %title("Trackink LQR Experimental",'interpreter','latex')

figure(1)
% Sample data
reference_path_x = xRef;
reference_path_y = yRef;
actual_path_x = xx3;
actual_path_y = yy3;
% Calculate error
%error = sqrt((actual_path_x - reference_path_x).^2 + (actual_path_y - reference_path_y).^2);
% Normalize error
normalized_error = mind3;%normalize(mind1);
% Create scatter plot
scatter(actual_path_x, actual_path_y, [], normalized_error, 'filled');
colorbar;
% plot(xx1,yy1,xRef,yRef,"LineWidth",1)
xlabel("X-axis (m)",'interpreter','latex')
ylabel("Y-axis (m)",'interpreter','latex')
xlim([-4, 4])
ylim([-1, 5])
title("Tracking LQR Experimental",'interpreter','latex')
set(gca, 'FontSize', 12);
hold on 
plot(xRef,yRef,'r','linewidth', 2)
grid on
set(gca, 'FontSize', 12);

%%%% plot states
figure(2)
plot(time1(204:939)-7,mind1(204:939),time2,mind2,time3(204:901)-7,mind3(204:901),time4(380:1105)-14,mind4(380:1105),"LineWidth",1)
legend('10-1-10-0.5-10','25-1-20-0.5-10','25-1-30-0.5-10','30-1-30-0.5-10','interpreter','latex')
xlabel("Time (s)",'interpreter','latex')
ylabel("Cross Track Error (m)",'interpreter','latex')
title("Tuning Q Matrix",'interpreter','latex')


%%%% plot states
figure(2)

subplot(2,2,1)
patch(time1,mind1,c1,'EdgeColor','interp',"LineWidth",1)
legend('10-1-10-0.5-10')
xlabel("Time (s)",'interpreter','latex')
ylabel("Look-ahead distance (m)",'interpreter','latex')
colorbar;
grid on
set(gca, 'FontSize', 12);

subplot(2,2,2)
patch(time2,mind2,c2,'EdgeColor','interp',"LineWidth",1)
legend('25-1-20-0.5-10')
xlabel("Time (s)",'interpreter','latex')
ylabel("Cross Track Error (m)",'interpreter','latex')
colorbar;
grid on
set(gca, 'FontSize', 12);

subplot(2,2,3)
patch(time3,mind3,c3,'EdgeColor','interp',"LineWidth",1)
legend('25-1-30-0.5-10')
xlabel("Time (s)",'interpreter','latex')
ylabel("Cross Track Error (m)",'interpreter','latex')
colorbar;
grid on
set(gca, 'FontSize', 12);

subplot(2,2,4)
patch(time4,mind4,c4,'EdgeColor','interp',"LineWidth",1)
legend('30-1-30-0.5-10')
xlabel("Time (s)",'interpreter','latex')
ylabel("Cross Track Error (m)",'interpreter','latex')
colorbar;
grid on
set(gca, 'FontSize', 12);
