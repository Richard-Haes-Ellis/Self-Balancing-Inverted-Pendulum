clear all
clc

K = [0 0 0];
Kfric = 1.5;
g = 9.81; % Gravity
ro = 2700; % Density of Aluminum
h = 0.01; % thickness of disc
deltaH = 0.24;
l = 0.12;  % Length from pivot to disc
R = 0.06; % Radius of disc
Re = R;
Ri = R-0.01;
md = (pi*(Re^2-Ri^2)*h*ro);
ms = 1.1*md;
Is = ms*(l^2);
%Id = (1/2)*md*R^2; % Solid Cylinder
Id = (1/2)*md*(Re^2+Ri^2); % Hollow Cylinder

t0 = 0;
tn = 10;

% State space 
A = [-((ms*g*l)/Kfric)  0          -(Is/Kfric);
     ((ms*g*l)/Is)       -Kfric/Is         0;
       0                ((ms*g*l)/Is)   -(Kfric/Is)];
B = [(Id/Is) -(Id/Is) 0]';
C = [1 0 0;
     0 1 0;
     0 0 0];
D = [0 0 0]';
Q = [10 0 0;
     0  0.1 0;
     0  0 0.1];
R = 0.1;

SYS = ss(A,B,C,D);

[K,S,e] = lqr(SYS,Q,R,0);

eig(A-B*K)



% -------------------------------------------------------------------

%Run the model 
sim('model2',[t0 tn]);


%-----------Static Plotting----------------

%Plot angular displacement of mass 
figure('rend','painters','pos',[100 100 1200 500])


        % Plot Velocity and Acceleration
        subplot(3,4,[1 2])
        plot(t,Pos)
        title('Angular Displacement of Mass                                ')
        xlabel('time (s)'); 
        ylabel('\theta (rad)');
        grid;
        subplot(3,4,[5 6])
        plot(t,MAcceleration)
        title('Motor Acceleration                                          ') 
        xlabel('time (s)'); 
        ylabel('\omega (rad/s^2)');
        grid;
        subplot(3,4,[9 10])
        plot(t,MTorque)
        title('Motor Torque                                                ')
        xlabel('time (s)'); 
        ylabel('\tau (Nm)');
        grid;

%--------------Dynamic Plotting--------------

% Create the markers representing the pendulum which will 
% Vary according to calculated positions
subplot(3,4,[3 4 7 8 11 12])
h=plot(0,0,'MarkerSize',10,'Marker','+','LineWidth',5); %the BC
hold on
h2=plot(0,0,'MarkerSize',300,'Marker','.','LineWidth',2,'Color','g');
h1=plot(0,0,'MarkerSize',20,'Marker','.','LineWidth',2,'Color','b'); %the mass

hold off

% Configure the axis to slightly larger than the lines for a suitable
% Animation space
range=1.5*l; axis([-range range -range range]); axis square;
title('Pendulum Animation')
xlabel('x displacement'); 
ylabel('y displacement');
grid;
set(gca,'nextplot','replacechildren');  %line to fresh each plot

for i=1:length(Pos)-1
    if (ishandle(h1)==1) % check figure is plotting
        % x cordinate of mass 
        Xcoord=[0,l*sin(Pos(i))];
        % y cordinate of mass 
        Ycoord=[0,l*cos(Pos(i))];
        % Update x and y cords
        set(h2,'XData',Xcoord(2),'YData',Ycoord(2));
        set(h1,'XData',Xcoord,'YData',Ycoord);
        drawnow;
    end
end




