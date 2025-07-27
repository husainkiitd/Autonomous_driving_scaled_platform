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
y4 = 0.0;

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





data1 = load("racetrackPP12march_ld_0.6.mat");
data2 = load("racetrackPP12march_ld_0.7.mat");
data3 = load("racetrackPP12march_ld_0.8.mat");
data4 = load("racetrackPP12march_ld_0.9.mat");
data5 = load("racetrackPP12march_ld_1.mat");
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
time1 =data1.out.pose.time(:,1);
xx1 =data1.out.pose.signals.values(:,1);
yy1 =data1.out.pose.signals.values(:,2);
theta1 =data1.out.pose.signals.values(:,3);
mind1 =data1.out.min_d.signals.values(:,:,1:length(time1));
mind1 = reshape(mind1, [], 1);
mind1(end) = NaN;
c1 = mind1;
rms1=rms(abs(mind1));
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
time2 =data2.out.pose.time(:,1);
xx2 =data2.out.pose.signals.values(:,1);
yy2 =data2.out.pose.signals.values(:,2);
theta2 =data2.out.pose.signals.values(:,3);
mind2 =data2.out.min_d.signals.values(:,:,1:length(time2));
mind2 = reshape(mind2, [], 1);
mind2(end) = NaN;
c2 = mind2;
rms2=rms(mind2);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
time3 =data3.out.pose.time(:,1);
xx3 =data3.out.pose.signals.values(:,1);
yy3 =data3.out.pose.signals.values(:,2);
theta3 =data3.out.pose.signals.values(:,3);
mind3 =data3.out.min_d.signals.values(:,:,1:length(time3));
mind3 = reshape(mind3, [], 1);
mind3(end) = NaN;
c3 = mind3;
rms3=rms(mind3);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
time4 =data4.out.pose.time(:,1);
xx4 =data4.out.pose.signals.values(:,1);
yy4 =data4.out.pose.signals.values(:,2);
theta4 =data4.out.pose.signals.values(:,3);
mind4 =data4.out.min_d.signals.values(:,:,1:length(time4));
mind4 = reshape(mind4, [], 1);
mind4(end) = NaN;
c4 = mind4;
rms4=rms(mind4);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
time5 =data5.out.pose.time(:,1);
xx5 =data5.out.pose.signals.values(:,1);
yy5 =data5.out.pose.signals.values(:,2);
theta5 =data5.out.pose.signals.values(:,3);
mind5 =data5.out.min_d.signals.values(:,:,1:length(time5));
mind5 = reshape(mind5, [], 1);
mind5(end) = NaN;
c5 = mind5;
rms5=rms(mind5);
%%%%%%%%%%%%%%%%%%%%%%%%%%%
rms1 = rms(mind1(~isnan(mind1)));
rms2 = rms(mind2(~isnan(mind2)));
rms3 = rms(mind3(~isnan(mind3)));
rms4 = rms(mind4(~isnan(mind4)));
rms5 = rms(mind5(~isnan(mind5)));

RMS = [rms1, rms2, rms3, rms4, rms5,]

%%%% plot trajectory


figure(1)

% Sample data
reference_path_x = xRef;
reference_path_y = yRef;
actual_path_x = xx1;
actual_path_y = yy1;
% Calculate error
%error = sqrt((actual_path_x - reference_path_x).^2 + (actual_path_y - reference_path_y).^2);
% Normalize error
normalized_error = mind1;%normalize(mind1);
% Create scatter plot
scatter(actual_path_x, actual_path_y, [], normalized_error, 'filled');
colorbar;
% plot(xx1,yy1,xRef,yRef,"LineWidth",1)
xlabel("X-axis (m)",'interpreter','latex')
ylabel("Y-axis (m)",'interpreter','latex')
xlim([-4, 4])
ylim([-1, 5])
title("Tracking Pure Pursuit Experimental",'interpreter','latex')
set(gca, 'FontSize', 12);
hold on 
plot(xRef,yRef,'r','linewidth', 2)
grid on
set(gca, 'FontSize', 12);

%%%% plot states
figure(2)
subplot(2,2,1)
patch(time1,mind1,c1,'EdgeColor','interp',"LineWidth",1)
legend('ld\_0.6')
xlabel("Time (s)",'interpreter','latex')
ylabel("Cross Track Error (m)",'interpreter','latex')
colorbar;
grid on
set(gca, 'FontSize', 12);

subplot(2,2,2)
patch(time2,mind2,c2,'EdgeColor','interp',"LineWidth",1)
legend('ld\_0.7')
xlabel("Time (s)",'interpreter','latex')
ylabel("Cross Track Error (m)",'interpreter','latex')
colorbar;
grid on
set(gca, 'FontSize', 12);

subplot(2,2,3)
patch(time3,mind3,c3,'EdgeColor','interp',"LineWidth",1)
legend('ld\_0.8')
xlabel("Time (s)",'interpreter','latex')
ylabel("Cross Track Error (m)",'interpreter','latex')
colorbar;
grid on
set(gca, 'FontSize', 12);

subplot(2,2,4)
patch(time4,mind4,c4,'EdgeColor','interp',"LineWidth",1)
legend('ld\_0.9')
xlabel("Time (s)",'interpreter','latex')
ylabel("Cross Track Error (m)",'interpreter','latex')
colorbar;
grid on
set(gca, 'FontSize', 12);


