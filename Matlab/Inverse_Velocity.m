clear all;
close all;

%% INITIAL ARM CONDITION
l1 = 1.2;
l2 = 1.5;
l3 = 1.35;

theta = 0;
beta = 0;
gamma = 0;
angle_init = [theta; beta; gamma];


%% ARM ERROR TERMS
global l1_err l2_err l3_err
l1_err = 0.1;
l2_err = -0.1;
l3_err = -0.1;
%% TRACING VECTOR FUNCTION
R = 1;

t = (0: pi/180: 2*pi);

dt = pi/180;
%% CIRCLE
% x = R*cos(t);
% y = R*sin(t);
% z = ones(1, length(x));
% 
% %%SPEED 
% x_dot = -R*sin(t);
% y_dot = R*cos(t);
% z_dot = zeros(1,length(x_dot));
% 
% %velocity magnitude
% v_amp = sqrt(x_dot.^2 + y_dot.^2 + z_dot.^2);
% 
% %velocity after normalization
% v_vec = [x_dot./v_amp; y_dot./v_amp; z_dot./v_amp;];
% 
% %velocity profile (parabolic)
% v_profile = (-1/pi)*t.^2+2*t;
% %velocity desired profile
% % v_vec = v_vec.*v_profile;

%% ELLIPSE
x = R*cos(t);
y = R*sin(t);
z = cos(t);

%velocity before normalization
x_dot = -R*sin(t);
y_dot = R*cos(t);
z_dot = -sin(t);

%velocity magnitude
v_amp = sqrt(x_dot.^2 + y_dot.^2 + z_dot.^2);

%velocity after normalization
v_vec = [x_dot./v_amp; y_dot./v_amp; z_dot./v_amp;];

%% SPIRAL
% R = 1.3;
% t = (0: pi/180: 4*pi);
% x = R*cos(t);
% y = R*sin(t);
% z = t./4;
% % MATH & ROBOT & ROMANCE
% x_old = 0.1*(13*cos(t)-5*cos(2*t)-2*cos(3*t)-cos(4*t));
% y_old = 0.1*(16*sin(t).^3);
% z = ones(1, length(x_old))+1*x_old;
% %%rotate the equation
% x = x_old.*cos(pi/4)-y_old.*sin(pi/4);
% y = x_old.*sin(pi/4)+y_old.*cos(pi/4);
%% DESIRED CORDINATED VECTOR
cord_vec = [x; y; z];


%% ANIMATION FOR THE ROBOTIC LINKAGES
figure(1)

%%CONTROL PARAMETERS
kp = [5, 0, 0;
      0, 5, 0;
      0, 0, 5];
ki = [0.8, 0, 0;
      0, 0.8, 0;
      0, 0, 0.8];

% kp = [0, 0, 0;
%       0, 0, 0;
%       0, 0, 0];
% ki = [0, 0, 0;
%       0, 0, 0;
%       0, 0, 0];

err_ki = [0; 0; 0];

%INITIALIZE ANGLE AT THE START OF THE CURVE
current_angle = cal_angle(cord_vec(:,1), angle_init, l1, l2, l3);

%ALLOCATING SPACE FOR END EFFECTOR LOCATION
end_x = zeros(1,length(t));
end_y = zeros(1,length(t));
end_z = zeros(1,length(t));

%% Initialize video
% myVideo = VideoWriter('NO_CONTROL'); %open video file
% myVideo.FrameRate = 60;  %can adjust this, 5 - 10 works well for me
% open(myVideo)

%ITERATING CONTROL PROCESS
for iteration = 1:1:length(t)
    [cart_x, cart_y, cart_z] = cal_cart_arm_real(current_angle,l1,l2,l3);
    %PLOT REAL CONFIG
    h = plot3(cart_x,cart_y,cart_z,'linewidth',1);  
    pbaspect([1 1 1])
    xlim([-4 4])
    ylim([-4 4])
    zlim([-4 4])
    title('Inverse Velocity Control')
    pause(.006)
    %CALCULATE REAL END EFFECTOR LOCATION
    cart_end = [cart_x(3,2); cart_y(3,2); cart_z(3,2)];
    
    %SAVE END EFFECTOR LOCATION FOR PLOTTING
    end_x(iteration) = cart_x(3,2);
    end_y(iteration) = cart_y(3,2);
    end_z(iteration) = cart_z(3,2);
    
    %POSITION ERROR
    err = cord_vec(:, iteration) - cart_end;
    norm(err)
    %INTEGRAL ERROR
    err_ki = err_ki +err;
    %THE INPUT VELOCITY FROM FEEDBACK CONTROL
    v_current = v_vec(:, iteration) + kp*err + ki*err_ki;
    %CALCULATE REAL VELOCITY
    v_current_real = cal_real_velocity(v_current, current_angle, l1, l2, l3);
    %END OF MOTION IN dt
    cart_motion_end = cart_end + dt*v_current_real;
    %CALCULATE CORRESPONDING ANGLE
    current_angle = cal_angle(cart_motion_end, current_angle, l1, l2, l3); 
    
    %GRAB FRAMES
%     frame = getframe(gcf); %get frame
%     writeVideo(myVideo, frame);    
end

hold on

%IDEAL PATH
plot3(x,y,z)
hold on
%ACTUAL PATH
plot3(end_x, end_y, end_z)

pbaspect([1 1 1])
xlim([-4 4])
ylim([-4 4])
zlim([-4 4])
title('Inverse Velocity Control')

%GRAB FRAMES
% for iteration = 1:1:100
%     frame = getframe(gcf); %get frame
%     writeVideo(myVideo, frame);
% end
% %CLOSE VIDEO
% close(myVideo)

%% RANDOM ANGLE MEASUREMENT ERROR
function angle_err = gen_angle_err()
    min = -0.25*pi/180;
    max = 0.25*pi/180;
    angle_err = (max - min)*rand(1,1)+min;
end


%% FUNCTION FOR RETURING REAL VELOCITY OUTPUT
function v_current_real = cal_real_velocity(v_current, current_angle, l1, l2, l3)
    %IDEAL PSEUDO INVERSE J
    inv_J = pseudo_J(current_angle(1),current_angle(2),current_angle(3),l1,l2,l3 );
    %ANGULAR SPEED INPUT
    w_current = inv_J*v_current;
    %REAL J 
    J_real = jacobian_real(current_angle(1),current_angle(2),current_angle(3), l1, l2, l3);
    %REAL CURRENT VELOCITY
    v_current_real = J_real*w_current;
end


%% FUNCTION FOR RETURNING REAL LOCATIONS OF JOINTS
function [cart_x, cart_y, cart_z] = cal_cart_arm_real(current_angle, l1, l2, l3)
    global l1_err l2_err l3_err
    %ACTUAL PARAMETERS
    l1 = l1 + l1_err;
    l2 = l2+ l2_err;
    l3 = l3+ l3_err;
    %%INITIATE ARM LOCATION 
    p0_x = [0];
    p0_y = [0];
    p0_z = [0];
    %%CALCULATE FIRST JOINT LOCATION
    p1_x = [0];
    p1_y = [0];
    p1_z = [l1];
    %%CALCULATE SECOND JOINT LOCATION
    p2_x = [l2*cos(current_angle(2))*cos(current_angle(1))];
    p2_y = [l2*cos(current_angle(2))*sin(current_angle(1))];
    p2_z = [l1 + l2*sin(current_angle(2))];
    %%CALCULATING THIRD JOINT LOCATION
    p3_x = [l3*(cos(current_angle(2))*cos(current_angle(3))*cos(current_angle(1)) - sin(current_angle(2))*cos(current_angle(1))*sin(current_angle(3))) + l2*cos(current_angle(2))*cos(current_angle(1))];
    p3_y = [l3*(cos(current_angle(2))*cos(current_angle(3))*sin(current_angle(1)) - sin(current_angle(2))*sin(current_angle(3))*sin(current_angle(1))) + l2*cos(current_angle(2))*sin(current_angle(1))];
    p3_z = [l1 + l3*(cos(current_angle(2))*sin(current_angle(3)) + cos(current_angle(3))*sin(current_angle(2))) + l2*sin(current_angle(2))];

    %%COMBINING FOR LINE PLOTTING
    cart_x = [p0_x, p1_x; p1_x, p2_x; p2_x, p3_x];
    cart_y = [p0_y, p1_y; p1_y, p2_y; p2_y, p3_y];
    cart_z = [p0_z, p1_z; p1_z, p2_z; p2_z, p3_z];
end
%% FUNCTION FOR CALCULATING DESIRED ANGLES
function angle_desired = cal_angle(pos_desired, angle_init, l1, l2, l3)
 theta_ite = angle_init;
 global l1_err l2_err l3_err
 %ACTUAL PARAMETERS
%  l1 = l1 + l1_err;
%  l2 = l2+ l2_err;
%  l3 = l3+ l3_err;
 %INITIATE DISTANCE ERROR
 pos_error = pos_desired - cart_cord(theta_ite(1,end),theta_ite(2,end),theta_ite(3,end),l1,l2,l3 );
 while (norm(pos_error)>0.001)
    inv_J = pseudo_J(theta_ite(1,end),theta_ite(2,end),theta_ite(3,end),l1,l2,l3 );
    del_angle = inv_J*pos_error;   
    theta_ite = [theta_ite, theta_ite(1:3, end) + del_angle];
    pos_error = pos_desired - cart_cord(theta_ite(1,end),theta_ite(2,end),theta_ite(3,end),l1,l2,l3 );
 end
 angle_desired = theta_ite(1:3, end);
end
%% FUNCTION FOR CALCULATING PSEUDO INVERSE OF JACOBIAN MATRIX
function pseudo_J = pseudo_J(theta, beta, gamma, l1, l2, l3)
  J = [ - l3*(cos(beta)*cos(gamma)*sin(theta) - sin(beta)*sin(gamma)*sin(theta)) - l2*cos(beta)*sin(theta), - l3*(cos(beta)*cos(theta)*sin(gamma) + cos(gamma)*sin(beta)*cos(theta)) - l2*sin(beta)*cos(theta), -l3*(cos(beta)*cos(theta)*sin(gamma) + cos(gamma)*sin(beta)*cos(theta));
      l3*(cos(beta)*cos(gamma)*cos(theta) - sin(beta)*cos(theta)*sin(gamma)) + l2*cos(beta)*cos(theta), - l3*(cos(beta)*sin(gamma)*sin(theta) + cos(gamma)*sin(beta)*sin(theta)) - l2*sin(beta)*sin(theta), -l3*(cos(beta)*sin(gamma)*sin(theta) + cos(gamma)*sin(beta)*sin(theta));
      0, l2*cos(beta) - l3*(sin(beta)*sin(gamma) - cos(beta)*cos(gamma)), -l3*(sin(beta)*sin(gamma) - cos(beta)*cos(gamma))];
  pseudo_J = pinv(J);
end

%% FUNCTION FOR CALCULATING CORRESPONDING CARTESIAN COORDINATES
function f = cart_cord(theta, beta, gamma, l1, l2, l3)
    f = [ l3*(cos(beta)*cos(gamma)*cos(theta) - sin(beta)*cos(theta)*sin(gamma)) + l2*cos(beta)*cos(theta);
      l3*(cos(beta)*cos(gamma)*sin(theta) - sin(beta)*sin(gamma)*sin(theta)) + l2*cos(beta)*sin(theta);
      l1 + l3*(cos(beta)*sin(gamma) + cos(gamma)*sin(beta)) + l2*sin(beta)];
end

%% FUNCTION FOR CALCULATING REAL JACOBIAN
function  J = jacobian_real(theta, beta, gamma, l1, l2, l3)
    global l1_err l2_err l3_err
    %ACTUAL PARAMETERS
    l1 = l1 + l1_err;
    l2 = l2+ l2_err;
    l3 = l3+ l3_err;
    theta = theta + gen_angle_err();
    beta = beta + gen_angle_err();
    gamma = gamma + gen_angle_err();
    
    J = [ - l3*(cos(beta)*cos(gamma)*sin(theta) - sin(beta)*sin(gamma)*sin(theta)) - l2*cos(beta)*sin(theta), - l3*(cos(beta)*cos(theta)*sin(gamma) + cos(gamma)*sin(beta)*cos(theta)) - l2*sin(beta)*cos(theta), -l3*(cos(beta)*cos(theta)*sin(gamma) + cos(gamma)*sin(beta)*cos(theta));
          l3*(cos(beta)*cos(gamma)*cos(theta) - sin(beta)*cos(theta)*sin(gamma)) + l2*cos(beta)*cos(theta), - l3*(cos(beta)*sin(gamma)*sin(theta) + cos(gamma)*sin(beta)*sin(theta)) - l2*sin(beta)*sin(theta), -l3*(cos(beta)*sin(gamma)*sin(theta) + cos(gamma)*sin(beta)*sin(theta));
          0, l2*cos(beta) - l3*(sin(beta)*sin(gamma) - cos(beta)*cos(gamma)), -l3*(sin(beta)*sin(gamma) - cos(beta)*cos(gamma))];
end

