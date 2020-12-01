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

%% TRACING VECTOR FUNCTION
R = 1;

t = (0: pi/180: 2*pi);
%% CIRCLE
% x = R*cos(t);
% y = R*sin(t);
% z = ones(1, length(x));
%% ELLIPSE
% x = R*cos(t);
% y = R*sin(t);
% z = cos(t);
%% SPIRAL
R = 1.3;
t = (0: pi/180: 4*pi);
x = R*cos(t);
y = R*sin(t);
z = t./4;
%% MATH & ROBOT & ROMANCE
% x_old = 0.1*(13*cos(t)-5*cos(2*t)-2*cos(3*t)-cos(4*t));
% y_old = 0.1*(16*sin(t).^3);
% z = ones(1, length(x_old))+1*x_old;
% %%rotate the equation
% x = x_old.*cos(pi/4)-y_old.*sin(pi/4);
% y = x_old.*sin(pi/4)+y_old.*cos(pi/4);
%% DESIRED CORDINATED VECTOR
cord_vec = [x; y; z];


%% ANIMATION FOR THE ROBOTIC LINKAGES
cal_angle([1;1;0],[0;0;0],l1,l2,l3)
figure(1)

%% Initialize video
% myVideo = VideoWriter('SPIRAL'); %open video file
% myVideo.FrameRate = 80;  %can adjust this, 5 - 10 works well for me
% open(myVideo)

%%INITIALIZE CURRENT ANGLE
current_angle = angle_init;
for iteration = 1:1:length(x)
    pos_desired = cord_vec(:,iteration);
    angle_desired = cal_angle(pos_desired, current_angle, l1, l2, l3);
    [cart_x, cart_y, cart_z] = cal_cart_arm(current_angle,l1,l2,l3);
    %%PLOT ARM
    h = plot3(cart_x,cart_y,cart_z,'linewidth',1);
    pbaspect([1 1 1])
    xlim([-4 4])
    ylim([-4 4])
    zlim([-4 4])
    title('Inverse Kinematics')
    %grid on 
    current_angle = angle_desired;
    pause(.006)
    
    %GRAB FRAMES
%     frame = getframe(gcf); %get frame
%     writeVideo(myVideo, frame);
end
hold on

plot3(x,y,z)

pbaspect([1 1 1])
xlim([-4 4])
ylim([-4 4])
zlim([-4 4])
title('Inverse Kinematics')

%GRAB FRAMES
% for iteration = 1:1:100
%     frame = getframe(gcf); %get frame
%     writeVideo(myVideo, frame);
% end
% %CLOSE VIDEO
% close(myVideo)

%% FUNCTION FOR RETURNING LOCATIONS OF JOINTS
function [cart_x, cart_y, cart_z] = cal_cart_arm(current_angle, l1, l2, l3)
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

%% FUNCTION FOR CALCULATING JACOBIAN
function  J = jacobian(theta, beta, gamma, l1, l2, l3)
    J = [ - l3*(cos(beta)*cos(gamma)*sin(theta) - sin(beta)*sin(gamma)*sin(theta)) - l2*cos(beta)*sin(theta), - l3*(cos(beta)*cos(theta)*sin(gamma) + cos(gamma)*sin(beta)*cos(theta)) - l2*sin(beta)*cos(theta), -l3*(cos(beta)*cos(theta)*sin(gamma) + cos(gamma)*sin(beta)*cos(theta));
          l3*(cos(beta)*cos(gamma)*cos(theta) - sin(beta)*cos(theta)*sin(gamma)) + l2*cos(beta)*cos(theta), - l3*(cos(beta)*sin(gamma)*sin(theta) + cos(gamma)*sin(beta)*sin(theta)) - l2*sin(beta)*sin(theta), -l3*(cos(beta)*sin(gamma)*sin(theta) + cos(gamma)*sin(beta)*sin(theta));
          0, l2*cos(beta) - l3*(sin(beta)*sin(gamma) - cos(beta)*cos(gamma)), -l3*(sin(beta)*sin(gamma) - cos(beta)*cos(gamma))];
end

