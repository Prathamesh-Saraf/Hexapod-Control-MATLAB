%close all
clear all
clc

%%%%%%%%%%%%%%% HEXAPOD DIMENSIONS %%%%%%%%%%%%%%%%
global x1256 y1256 x34 y34 hx1256 hy1256 hx34 hy34 a1 a2 a3
a1 = 5;a2 =5;a3 =5;hx34 = 4;hy34 = 0;hx1256 = hx34*cos(pi/3);hy1256 = hx34*sin(pi/3);x34 = hx34+a2;y34 = 0;x1256 = x34*cos(pi/3);y1256 = x34*sin(pi/3);

%%%%%%%%%%%% ROLL, PITCH & YAW ANGLES %%%%%%%%%%%%%
roll = pi/12; pitch = pi/12; yaw = pi/12;

%%%%%%%%%%%% MOTION CALL FUNCTIONS %%%%%%%%%%%%%%%
% for i=0:1:5
%     yaw_motion = rpy(0,0,yaw,0);pause(0.25)
%     yaw_motion1 = rpy(0,0,0,0);pause(0.1)
%     yaw_motion2 = rpy(0,0,-yaw,0);pause(0.25)
%     yaw_motion3 = rpy(0,0,0,0);pause(0.1)
% end
% for i=0:1:5
%     updown = rpy(0,0,0,1);pause(0.25)
%     updown1 = rpy(0,0,0,0);pause(0.1)
%     updown2 = rpy(0,0,0,-1);pause(0.25)
%     updown3 = rpy(0,0,0,0);pause(0.1)
% end
% % for i=0:1:5
% %     roll_motion = rp(roll,0);pause(0.25)
% %     roll_motion1 = rp(0,0);pause(0.1)
% %     roll_motion2 = rp(-roll,0);pause(0.25)
% %     roll_motion, 3 = rp(0,0);pause(0.1)
% % end
for i=0:1:5
    pitch_motion = rp(0,pitch);pause(0.25)
    pitch_motion1 = rp(0,0);pause(0.1)
%     pitch_motion2 = rp(0,-pitch);pause(0.25)
%     pitch_motion3 = rp(0,0);pause(0.1)
end
% for i=0:1:5
%     circular_motion = rp(0,pitch);pause(0.25)
%     circular_motion1 = rp(roll,0);pause(0.1)
%     circular_motion2 = rp(0,-pitch);pause(0.25)
%     circular_motion3 = rp(-roll,0);pause(0.1)
% end
% for i=0:1:1
%     roll_motion = rpy(roll,0,0,0);pause(0.25)
%     roll_motion1 = rpy(0,0,0,0);pause(0.1)
%     roll_motion2 = rpy(-roll,0,0,0);pause(0.25)
%     roll_motion3 = rpy(0,0,0,0);pause(0.1)
% end
% for i=0:1:1
%     pitch_motion = rpy(0,pitch,0,0);pause(0.25)
%     pitch_motion1 = rpy(0,0,0,0);pause(0.1)
% %     pitch_motion2 = rpy(0,-pitch,0,0);pause(0.25)
% %     pitch_motion3 = rpy(0,0,0,0);pause(0.1)
% end
% for i=0:1:1
%     circular_motion = rpy(0,pitch,0,0);pause(0.25)
%     circular_motion1 = rpy(roll,0,0,0);pause(0.1)
%     circular_motion2 = rpy(0,-pitch,0,0);pause(0.25)
%     circular_motion3 = rpy(-roll,0,0,0);pause(0.1)
% end    
% y2=0;y1=-1;
% for i=0:2:20
%     for j=-1:0.075:0.9
%         z = sqrt(1 - j^2);
%         if (abs(j)<= 0.75)
%             t = 0.01;
%         else
%             t = 0.5;
%         end
%         if mod(i,4) == 0
%             tripod1 = tripod(y1,y2,z,0,i+j);pause(t)
%             y1=i+j;
%         else
%             tripod2 = tripod(y1,y2,0,z,i+j);pause(t)
%             y2=i+j;
%         end
%     end
% end

%%%%%%%%%%%%%%% PURE TRANSLATION OF TORSO & YAW %%%%%%%%%%%%%%%%%
function plot1 = rpy(roll_angle,pitch_angle,yaw_angle,z)
global x1256 y1256 x34 y34 a1
c = a1*tan(roll_angle); d = a1*tan(pitch_angle);

y11 = d+4*sin(yaw_angle+(pi/3));y21 = d+4*sin(yaw_angle+(2*pi/3));y31 = d+4*sin(yaw_angle);y41 = d+4*sin(yaw_angle+(pi));y51 = d+4*sin(yaw_angle+(5*pi/3));y61 = d+4*sin(yaw_angle+(4*pi/3));
x11 = c+sqrt(16 - (y11-d)^2);x21 = c-sqrt(16 - (y21-d)^2);x31 = c+sqrt(16 - (y31-d)^2);x41 = c-sqrt(16 - (y41-d)^2);x51 = c+sqrt(16 - (y51-d)^2);x61 = c-sqrt(16 - (y61-d)^2);
z11 = a1 + z; z21 = a1 + z; z31 = a1 + z; z41 = a1 + z; z51 = a1 + z; z61 = a1 + z;

[T1, T2, T3] = invk(x1256-x11,y1256-y11,0,z11);
theta11 = T1; theta12 = T2; theta13 = T3;  
[T1, T2, T3] = invk(x1256+x21,y1256-y21,0,z21);
theta21 = T1; theta22 = T2;theta23 = T3; 
[T1, T2, T3] = invk(x34-x31,y34-y31,0,z31);
theta31 = T1; theta32 = T2; theta33 = T3; 
[T1, T2, T3] = invk(x34+x41,y34-y41,0,z41);
theta41 = T1; theta42 = T2; theta43 = T3;
[T1, T2, T3] = invk(x1256-x51,y1256+y51,0,z51);
theta51 = T1; theta52 = T2; theta53 = T3;
[T1, T2, T3] = invk(x1256+x61,y1256+y61,0,z61);
theta61 = T1; theta62 = T2; theta63 = T3;
t11 = theta11 - 1.0472, t12 = theta12, t13 = theta13,
t21 = theta21 - 1.0472, t22 = theta22, t23 = theta23,
t31 = theta31, t32 = theta32, t33 = theta33,
t41 = theta41, t42 = theta42, t43 = theta43,
t51 = theta51 - 1.0472, t52 = theta52, t53 = theta53,
t61 = theta61 - 1.0472, t62 = theta62, t63 = theta63,
plot1 = plot(x11,y11,z11,x21,y21,z21,x31,y31,z31,x41,y41,z41,x51,y51,z51,x61,y61,z61,theta11,theta12,theta13,theta21,theta22,theta23,theta31,theta32,theta33,theta41,theta42,theta43,theta51,theta52,theta53,theta61,theta62,theta63);
end

%%%%%%%%%%%%%%% ROLL, PITCH %%%%%%%%%%%%%%%%%
function plot1 = rp(roll_angle,pitch_angle)
global hx1256 hy1256 hx34 hy34 x1256 y1256 x34 y34 a1

z11 = a1+hy1256*sin(roll_angle)+hx1256*sin(pitch_angle); z21 = a1+hy1256*sin(roll_angle)-hx1256*sin(pitch_angle); z31 = a1+hx34*sin(pitch_angle); z41 = a1-hx34*sin(pitch_angle); z51 = a1-hy1256*sin(roll_angle)+hx1256*sin(pitch_angle); z61 = a1-hy1256*sin(roll_angle)-hx1256*sin(pitch_angle);
y11 = hy1256*cos(roll_angle);y21 = hy1256*cos(roll_angle);y31 = hy34;y41 = hy34;y51 = -hy1256*cos(roll_angle);y61 = -hy1256*cos(roll_angle);
x11 = hx1256*cos(pitch_angle);x21 = -hx1256*cos(pitch_angle);x31 = hx34*cos(pitch_angle);x41 = -hx34*cos(pitch_angle);x51 = hx1256*cos(pitch_angle);x61 = -hx1256*cos(pitch_angle);

[T1, T2, T3] = invk(x1256-x11,y1256-y11,0,z11);
theta11 = T1; theta12 = T2; theta13 = T3;  
[T1, T2, T3] = invk(x1256+x21,y1256-y21,0,z21);
theta21 = T1; theta22 = T2; theta23 = T3; 
[T1, T2, T3] = invk(x34-x31,y34-y31,0,z31);
theta31 = T1; theta32 = T2; theta33 = T3; 
[T1, T2, T3] = invk(x34+x41,y34-y41,0,z41);
theta41 = T1; theta42 = T2; theta43 = T3;
[T1, T2, T3] = invk(x1256-x51,y1256+y51,0,z51);
theta51 = T1; theta52 = T2; theta53 = T3;
[T1, T2, T3] = invk(x1256+x61,y1256+y61,0,z61);
theta61 = T1; theta62 = T2; theta63 = T3;
t11 = theta11 - 1.0472, t12 = theta12, t13 = theta13,
t21 = theta21 - 1.0472, t22 = theta22, t23 = theta23,
t31 = theta31, t32 = theta32, t33 = theta33,
t41 = theta41, t42 = theta42, t43 = theta43,
t51 = theta51 - 1.0472, t52 = theta52, t53 = theta53,
t61 = theta61 - 1.0472, t62 = theta62, t63 = theta63,
plot1 = plot(x11,y11,z11,x21,y21,z21,x31,y31,z31,x41,y41,z41,x51,y51,z51,x61,y61,z61,theta11,theta12,theta13,theta21,theta22,theta23,theta31,theta32,theta33,theta41,theta42,theta43,theta51,theta52,theta53,theta61,theta62,theta63);
end

%%%%%%%%%%%%%%% TRIPOD GAIT %%%%%%%%%%%%%%%%%
function plot1 = tripod(y1,y2,z1,z2,p)
global x1256 y1256 x34 y34 a1
kx = 2;ky = 2*sqrt(3); k = 4;
[T1, T2, T3] = invk(x1256-kx,y1256-ky-p+y1,z1,a1);
theta11 = T1; theta12 = T2; theta13 = T3;  
[T1, T2, T3] = invk(x1256-kx,y1256-ky-p+y2,z2,a1);
theta21 = T1; theta22 = T2; theta23 = T3; 
[T1, T2, T3] = invk(x34-k,y34-p+y2,z2,a1);
theta31 = T1; theta32 = T2; theta33 = T3; 
[T1, T2, T3] = invk(x34-k,y34-p+y1,z1,a1);
theta41 = T1; theta42 = T2; theta43 = T3;
[T1, T2, T3] = invk(x1256-kx,y1256-ky+p-y1,z1,a1);
theta51 = T1; theta52 = T2; theta53 = T3;
[T1, T2, T3] = invk(x1256-kx,y1256-ky+p-y2,z2,a1);
theta61 = T1; theta62 = T2; theta63 = T3;

x11 = kx;x21 = -kx;x31 = k;x41 = -k;x51 = kx;x61 = -kx;
y11 = ky+p;y21 = ky+p;y31 = p;y41 = p;y51 = -ky+p;y61 = -ky+p;
z11 = a1; z21 = a1; z31 = a1; z41 = a1; z51 = a1; z61 = a1;

plot1 = plot(x11,y11,z11,x21,y21,z21,x31,y31,z31,x41,y41,z41,x51,y51,z51,x61,y61,z61,theta11,theta12,theta13,theta21,theta22,theta23,theta31,theta32,theta33,theta41,theta42,theta43,theta51,theta52,theta53,theta61,theta62,theta63);
end

%%%%%%%%%%%%%%% INVERSE KINEMATICS %%%%%%%%%%%%%%%%%
function [T1, T2, T3] = invk(x,y,z,a)
global a2 a3
a1 = a;
r1 = sqrt(x^2 + y^2);
r2 = a1 - z;
if(r1== 0 && r2>=0)
    phi2 = pi/2;
elseif(r1== 0 && r2<0)
    phi2 = -pi/2;
else
    phi2 = atan(r2/r1);
end

r3 = sqrt(r1^2 + r2^2);
if(r3== 0)
    phi1 = 0;
else
    phi1 = acos((a3^2 - a2^2 - r3^2)/(-2*a2*r3));
end

phi3 = acos((r3^2 - a2^2 - a3^2)/(-2*a2*a3));

if(y == 0)
    T1 = 0;
else
    T1 = atan(y/x);
end

T2 = -(phi2 - phi1);
T3 = -(pi - phi3);
end

%%%%%%%%%%%%%%% PLOT FUNCTION %%%%%%%%%%%%%%%%%
function x11 = plot(x11,y11,z11,x21,y21,z21,x31,y31,z31,x41,y41,z41,x51,y51,z51,x61,y61,z61,theta11,theta12,theta13,theta21,theta22,theta23,theta31,theta32,theta33,theta41,theta42,theta43,theta51,theta52,theta53,theta61,theta62,theta63)
global a2 a3

x12 = a2*cos(theta11)*cos(theta12)+x11;y12 = a2*sin(theta11)*cos(theta12)+y11;z12 = a2*sin(theta12) + z11;
x22 = -a2*cos(theta21)*cos(theta22)+x21;y22 = a2*sin(theta21)*cos(theta22)+y21;z22 = a2*sin(theta22) + z21;
x32 = a2*cos(theta31)*cos(theta32)+x31;y32 = a2*sin(theta31)*cos(theta32)+y31;z32 = a2*sin(theta32) + z31;
x42 = -a2*cos(theta41)*cos(theta42)+x41;y42 = a2*sin(theta41)*cos(theta42)+y41;z42 = a2*sin(theta42) + z41;
x52 = a2*cos(theta51)*cos(theta52)+x51;y52 = -(a2*sin(theta51)*cos(theta52)-y51);z52 = a2*sin(theta52) + z51;
x62 = -(a2*cos(theta61)*cos(theta62)-x61);y62 = -(a2*sin(theta61)*cos(theta62)-y61);z62 = a2*sin(theta62) + z61;

x13 = a3*cos(theta11)*cos(theta12+theta13) + x12;y13 = a3*sin(theta11)*cos(theta12+theta13) + y12;z13 = a3*sin(theta12+theta13) + z12;
x23 = -(a3*cos(theta21)*cos(theta22+theta23) - x22);y23 = a3*sin(theta21)*cos(theta22+theta23) + y22;z23 = a3*sin(theta22+theta23) + z22;
x33 = a3*cos(theta31)*cos(theta32+theta33) + x32;y33 = a3*sin(theta31)*cos(theta32+theta33) + y32;z33 = a3*sin(theta32+theta33) + z32;
x43 = -(a3*cos(theta41)*cos(theta42+theta43) - x42);y43 = (a3*sin(theta41)*cos(theta42+theta43) + y42);z43 = a3*sin(theta42+theta43) + z42;
x53 = a3*cos(theta51)*cos(theta52+theta53) + x52;y53 = -(a3*sin(theta51)*cos(theta52+theta53) - y52);z53 = a3*sin(theta52+theta53) + z52;
x63 = -(a3*cos(theta61)*cos(theta62+theta63) - x62);y63 = -(a3*sin(theta61)*cos(theta62+theta63) - y62);z63 = a3*sin(theta62+theta63) + z62;

plot3([x11 x21],[y11 y21],[z11 z21],'ro-',[x21 x41],[y21 y41],[z21 z41],'ro-',[x41 x61],[y41 y61],[z41 z61],'ro-',...
      [x61 x51],[y61 y51],[z61 z51],'ro-',[x51 x31],[y51 y31],[z51 z31],'ro-',[x31 x11],[y31 y11],[z31 z11],'ro-',...
      [x11 x12],[y11 y12],[z11 z12],'k',[x12 x13],[y12 y13],[z12 z13],'ko-',...
      [x21 x22],[y21 y22],[z21 z22],'k',[x22 x23],[y22 y23],[z22 z23],'ko-',...
      [x31 x32],[y31 y32],[z31 z32],'k',[x32 x33],[y32 y33],[z32 z33],'ko-',...
      [x41 x42],[y41 y42],[z41 z42],'k',[x42 x43],[y42 y43],[z42 z43],'ko-',...
      [x51 x52],[y51 y52],[z51 z52],'k',[x52 x53],[y52 y53],[z52 z53],'ko-',...
      [x61 x62],[y61 y62],[z61 z62],'k',[x62 x63],[y62 y63],[z62 z63],'ko-',...
      'linewidth', 4)
  
axis([-20 20 -10 30 -0.1 20])
xlabel('x'); ylabel('y'); zlabel('z');
end