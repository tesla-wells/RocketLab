close all;

tspan = 0:0.02:20; % first few seconds of flight
y0 = [0; 0; 0; 0; pi/2; 0; 0];
[t,y] = ode45(@rockint2,tspan,y0);
pos_xs = y(:,1);
vel_xs = y(:,2);
pos_ys = y(:,3);
vel_ys = y(:,4);
thetas = y(:,5);

dydts = zeros(size(y));

for i = 1:numel(tspan)
    dydts(i,:) = rockint2(t(i),y(i,:));
end

acc_xs = dydts(:,2);
acc_ys = dydts(:,4);

figure;
plot(pos_xs,pos_ys)
title('Rocket trajectory')
xlabel('Distance downrange (m)')
ylabel('Altitude (m)')

disp(pos_xs(end))
disp(pos_ys(end))
disp(rad2deg(thetas(end)))

figure;
quiver(pos_xs,pos_ys,cos(thetas),sin(thetas))
title('Rocket trajectory and facing')
xlabel('Distance downrange (m)')
ylabel('Altitude (m)')

% figure;
% plot(t,vel_xs,t,vel_ys,t,acc_xs,t,acc_ys)

figure;
plot(t,rad2deg(thetas))
title('Rocket angle')
xlabel('Mission elapsed time (sec)')
ylabel('Rocket facing (deg)')