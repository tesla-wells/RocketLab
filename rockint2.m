% 3-DOF launch vehicle flight integrator
function dydt = rockint2(t,y)
pos_x = y(1);
vel_x = y(2);
pos_y = y(3);
vel_y = y(4);
theta = y(5);
theta_dot = y(6);
theta_int = y(7); % integral term for PID controller.

g0 = 9.81;
rho = 1.225;
mass = 10;
A = 0.05^2*pi; % 10 cm across, thus 127 cm tall (assuming density of water, as most fuels are close to)
CD = 0.75;
length = 1.27;
mom_inert = (1/12)*mass*(3*0.05^2+1.27^2);
stat_marg = -0.1; % how far behind the CG is the CP?  (if negative, the CP is in front and the rocket is unstable!)

thrust = 200; % TWR ~ 2

crosswind_x = 2; % 2 m/s ~ 5 mph wind

airspeed_x = vel_x+crosswind_x;
airspeed_y = vel_y;

airspeed = sqrt(airspeed_x^2+airspeed_y^2);
rel_wind = atan2(airspeed_y,airspeed_x); % angle of relative wind

drag = 0.5*rho*CD*A*airspeed^2;

drag_x = -drag*cos(rel_wind);
drag_y = -drag*sin(rel_wind);

theta_goal = pi/2; % 90 degrees, straight up.
theta_err = theta-theta_goal;

% Adjust these parameters:
kp = 0;
ki = 0;
kd = 0;

thrust_vec = kp*theta_err + ki*theta_int + kd*theta_dot;

thrust_x = (thrust/mass)*cos(theta + thrust_vec);
thrust_y = (thrust/mass)*sin(theta + thrust_vec);

acc_x = thrust_x + drag_x;
acc_y = thrust_y + drag_y - g0;

theta_dotdot = (stat_marg/mom_inert)*(-drag_y*cos(theta)+drag_x*sin(theta)) +...
    ((length/2)/mom_inert)*(-thrust_y*cos(theta)+thrust_x*sin(theta));

dydt = [vel_x; acc_x; vel_y; acc_y; theta_dot; theta_dotdot; theta_err];

end