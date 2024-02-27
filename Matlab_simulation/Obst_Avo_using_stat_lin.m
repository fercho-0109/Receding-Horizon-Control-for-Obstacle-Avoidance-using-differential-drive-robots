% replay the experiment of paper ACC2023
% Obstacle avoidance RHC for constrained differential drive 
%
%-----SIMULATION IN MATLAB USIN QUANCER QBot2 ENVIROMENT--------------------------
%
% #################################################################################
% Edited by Alexis Marino (mrnlsf96p01z605k@studenti.unical.it) September, 26th 2023
% #################################################################################
%
% Took form the article "An Obstacle-Avoidance Receding Horizon Control Scheme for Constrained 
% Differential-Drive Robot via Dynamic Feedback Linearization"
% Autors Cristian Tiriolo, Giuseppe Franz`e, Walter Lucia

% ------ change of linearization procedure to static I/O linearization---------------------------


clear
close all;
clc;

%% parameters

R=0.035;        % radius od wheels [m]
D= 0.235;      % distance between the two wheels [m]
omega=10;   % max angular velocity [rad/s] 
Ts= 0.15;       % sampling time [s]
db=0.5;        % safe distance radius [m]
E=0.3;          % tolerance [m] 
b=0.1;          % distance point b [m]

parameters=[R D omega Ts db  E b];

%% Robot model with Dynamic linearizton and imput constrain

% symetric saturations constrains
Hd=[-1/omega 0 1/omega 0;
    0 -1/omega 0 1/omega]';

T=[R/2 R/2;     % diff drive to unicycle
    R/D -R/D];

Tinv=inv(T);

Qbd=[db*db 0  % obstacle free ball shapping matrix 
    0 db*db];

Hu=Hd*Tinv;


% positions
p0=[0;0.25;0];      % robot initial position
X0=p0;
pf=[1.20;2.35;pi];    % robot final position
%pf=[2;1;pi];    % robot final position

% matrices for the model 
A=[1 0
    0 1];
B=[Ts 0
    0 Ts];


Qe=eye(2);
rho=0.01;
Qu=rho*eye(2);



%% creating the map and planner
load mapa.mat
map1=map;
%map = occupancyMap(complexMap,resolution=2);
inflate(map1,0.5)    % to enlarge the obstacles
validator = validatorOccupancyMap(stateSpaceSE2);
validator.Map = map1;
validator.ValidationDistance = 0.01; % interval for cheking state validity   Inf (default)
planner = plannerHybridAStar(validator,'interpolationDistance',0.2,'MinTurningRadius',1);
planner.NumMotionPrimitives=3;
startPose = p0'; % [meters, meters, radians]
goalPose = pf';
path = plan(planner,startPose,goalPose);%,SearchMode='exhaustive');
figure(1)
show(planner)
title('obstacles with biger dimention')
xp=path.States(:,1);
yp=path.States(:,2);
thetap=path.States(:,3);
n_path=length(double(xp));
% reference for the point b
xpb=xp+b*cos(thetap);
ypb=yp+b*sin(thetap);

%% Robot position "odometric calculation"
% X_robot_k1 =@(X_k,v_k,theta_k)(X_k+Ts*v_k*cos(theta_k));
% Y_robot_k1 =@(Y_k,v_k,theta_k)(Y_k+Ts*v_k*sin(theta_k));
% theta_k1 = @(theta_k,w_k)(theta_k+Ts*w_k);

%% control 
%___________off line________% 

% current states of the robot 
X_k=p0(1);     
Y_k=p0(2);
theta_k=p0(3);

% taken the reference points
xr=xp(1);     
yr=yp(1);
thetar=thetap(1);

% generating a referecne for point b
xrb=xpb(1);
yrb=ypb(1);

% compute the radius of the worse case scenario
ru=(2*omega*R*b)/(sqrt(4*b*b+D*D));

% Error computing acording to the system with respect to the reference
Err0=p0(1:2,1)-[xp(1);yp(1)];


% control computing of the worse case scenario
[Q0,Y0]=worse_case_stc_lin(Err0,A,B,ru,Qbd,Qe,Qu);
%[Q0,Y0]=worse_case_stc_lin_mincx(Err0,A,B,ru,Qbd,Qe,Qu)
%Q0=Qbd;
%% on-line part

% variables to plots the results 
x=[];
y=[];
theta=[];
wr=[];
wl=[];

% plot the robot movement in the enviroment  
figure(2)
load mapa.mat
show(map)
hold on
title('Robot movement with elipsoidals ')
% Start state
scatter(startPose(1,1),startPose(1,2),"g","filled")
% Goal state
scatter(goalPose(1,1),goalPose(1,2),"r","filled")

j=1;
k=1;
qseq=p0;
plot_ell=true;
plot_err= false;
serr=[];
terr=[];
Xerr=[];
Yerr=[];
avr=0;
%%

for i=0:Ts:10000
    % if  norm([X_k,Y_k]-[xp(k),yp(k)])<=E
    %     k=k+1;
    %     xr=xp(k);
    %     yr=yp(k);
    %     %for the point b
    %     xrb=xpb(k);
    %     yrb=ypb(k);
    % end
    % 
      % if norm([X_k,Y_k]-[xr,yr])<=E
      %   if k<n_path
      %       k=k+1;
      %       xr=xp(k);
      %       yr=yp(k);
      %   elseif k==n_path
      %       xr=xp(k);
      %       yr=yp(k);
      %       E=0.005;
      %       k=k+1;
      %   else
      %       break
      %   end
      % end
     dist=norm([X_k,Y_k]-[xr,yr]);
            terr=[terr,i];
            serr=[serr,dist];
            avr=avr+dist;
   dist2=norm([X_k+b*cos(theta_k),Y_k+b*sin(theta_k)]-[xr+b*cos(thetar),yr+b*sin(thetar)]);


    Err = [X_k+b*cos(theta_k);Y_k+b*sin(theta_k)]-[xr+b*cos(thetar);yr+b*sin(thetar)]; % error for the point b

    %Err = [X_k;Y_k]-[xr;yr]; %[[xr;yr]-[X_k;Y_k];0;0] 
    Xerr=[Xerr,Err(1)];
    Yerr=[Yerr,Err(2)];
    % Err2 = [X_k;Y_k]-[xr;yr]; %[[xr;yr]-[X_k;Y_k];0;0] 
    % Xerr=[Xerr,Err2(1)];
    % Yerr=[Yerr,Err2(2)];

    Tflinv = [cos(theta_k) sin(theta_k);
            -sin(theta_k)/b cos(theta_k)/b];

    H_theta = Hu*Tflinv;

   [Q_str,Y_str] =dyn_FL_kothare_constraint_OA(A,B,Err,Qe,Qu,H_theta,Q0);

   %[Q_str,Y_str] = RHC_OA_stc_lin_mincx2(Err,A,B,H_theta,Qbd,Qe,Qu); % with Kotare costarins 

    u = Tinv*Tflinv*Y_str*inv(Q_str)*Err;
    wr=[wr,u(1)];
    wl=[wl,u(2)];

    u2 = Tflinv*Y_str*inv(Q_str)*Err;

    % % Apply to the real plant 
    % x_real = (R/2)*(u(1)+u(2))*cos(theta_k);
    % y_real = (R/2)*(u(1)+u(2))*sin(theta_k);
    % theta_real = (R/D)*(u(1)-u(2));

  % continuos time-------------------------------------
    % Apply the control law to the non linear system
    v1=u2(1); w1=u2(2);
    t=0:0.00001:Ts;
    [t,q]= ode45(@(t,q,v,w)DiffDrive(t,q,v1,w1),t,qseq(:,end));
    %Update the state sequence 
    qseq=[qseq q(end,:)'];
    x(j)=X_k;
    X_k=qseq(1,end);
    y(j)=Y_k;
    Y_k=qseq(2,end);
    theta(j)=theta_k;
    theta_k=qseq(3,end);
    % --------------------------------------------------------------------

 
    
    % for ploting the elipsoidals  
    ell=ellipsoid([xr; yr],Q_str);
    B_d=ellipsoid([xr; yr],Q0);

    % if plot_err== true && (mod(j,10)==0 || j==1)   
    %     err=norm([X_k,Y_k]-[xr,yr]);
    %     serr=[serr,err];
    %     terr=[terr,i];
    % end

    if plot_ell== true && (mod(j,5)==0 || j==1)
        
        plot(projection(ell,[1 0; 0 1]));
        plot(projection(B_d,[1 0; 0 1]),'g');

    end
    %plotto la traiettoria del robot
    pause(0.001)
    %p_traj=plot(qseq(1,end),qseq(2,end),'b.'); if i use continuos time fashion
    p_traj=plot(X_k,Y_k,'b.');


      if dist2<=E
        if k<n_path
            k=k+1;
            xr=xp(k);
            yr=yp(k);
            thetar=thetap(k);
            xrb=xpb(k);
            yrb=xpb(k);

        elseif k==n_path
            xr=xp(k);
            yr=yp(k);
            thetar=thetap(k);
            xrb=xpb(k);
            yrb=xpb(k);
            E=0.05;
            k=k+1;
        else
            break
        end
      end

    j=j+1
end
avr=avr/j;
hold off
%%
plots=0;
if plots==1
figure(3)
show(map)
hold on
plot(x,y,'b',xp,yp,'--')
grid on 
title('Robot vs Reference')
hold off

% figure(4)
% plot(terr,Xerr,'r')
% grid on
% title('error in X')
% xlim([0 i]);
% 
% figure(5)
% plot(terr,Yerr,'r')
% grid on
% title('error in X')
% xlim([0 i]);
% 
% figure(6)
% plot(terr,wr,'b')%,t,wl,'b')
% legend('wr input')%,'wl')
% grid on
% title('wr ')%and wl inputs')
% xlim([0 i]);
% 
% figure(7)
% plot(terr,wl,'b')%,t,wl,'b')
% legend('wl')%,'wl')
% grid on
% title('wl input')%and wl inputs')
% xlim([0 i]);
% 
% figure(8)
% plot(terr,serr,'b')%,t,wl,'b')
% grid on
% title('Error tracking waypoint')%and wl inputs')
% xlim([0 i]);
end
if plots==1
    figure(1)
show(map1)
title('Quanser lab environment')
hold on
p=plot(x,y,'b',xp,yp,'r--');
p(1).LineWidth = 1.25;
p(2).LineWidth = 0.9;
legend(p,{'$Ref$','$Robot$'},'Interpreter','latex');

figure(2)
p1=plot(terr,wr,'b',terr,ones(1,length(terr))*omega,'--');
p1(2).Color='k';
l1=legend(p1,{'$\omega_R(t)$','$\omega_{R,max}$'},'Interpreter','latex');
l1.set('FontSize',15)
grid;
xlbl1=xlabel('$t[sec]$','Interpreter','latex');
ylbl1=ylabel('$\omega_R(t)[RAD/sec]$','Interpreter','latex');
ylbl1.FontSize=13;
xlbl1.FontSize=13;
axis([0 39 -10.5 10.5])

figure(3)
p1=plot(t1,wl,'b',t1,ones(1,length(t1))*omega,'--');
p1(2).Color='k';
l1=legend(p1,{'$\omega_L(t)$','$\omega_{L,max}$'},'Interpreter','latex');
l1.set('FontSize',15)
grid;
xlbl1=xlabel('$t[sec]$','Interpreter','latex');
ylbl1=ylabel('$\omega_L(t)[RAD/sec]$','Interpreter','latex');
ylbl1.FontSize=13;
xlbl1.FontSize=13;
axis([0 39 -10.5 10.5])

figure(4)
p=plot(t1,theta,'b',t1,thetarr,'r--');
title('$\theta-Robot$, versus $\theta-Ref$','Interpreter','latex');
p(1).LineWidth = 1.25;
p(2).LineWidth = 0.9;
l1=legend(p,{'$\theta Robot$','$\theta Ref$'},'Interpreter','latex');
l1.set('FontSize',15)
xlbl1=xlabel('$t[sec]$','Interpreter','latex');
ylbl1=ylabel('$\theta [RAD]$','Interpreter','latex');
ylbl1.FontSize=13;
xlbl1.FontSize=13;
axis([0 39 -2 6])

figure(5)
p=plot(t1,trackerr-0.3,'b',t1,cero,'k--');
title('Tracking error','Interpreter','latex')
p(1).LineWidth = 1;
p(2).LineWidth = 0.9;
l1=legend(p,{'track err','$0$'},'Interpreter','latex');
l1.set('FontSize',15)
xlbl1=xlabel('$t[sec]$','Interpreter','latex');
ylbl1=ylabel('$Err [m]$','Interpreter','latex');
ylbl1.FontSize=13;
xlbl1.FontSize=13;
axis([0 39 -0.05 0.6])
end