load mapa.mat
x=out.x.Data;
y=out.y.Data;
t1=out.wr.Time;
wr=out.wr.Data;
wl=out.wl.Data;
trackerr=out.err.Data;
theta=out.theta.Data;
xrr=xp;
yrr=yp;
thetarr=thetap;
figure(1)
show(map)
title('Quanser lab environment')
hold on
p=plot(x,y,'b',xrr,yrr,'r--');
p(1).LineWidth = 1.25;
p(2).LineWidth = 0.9;
legend(p,{'$Robot$','$Ref$'},'Interpreter','latex');

figure(2)
p1=plot(t1,wr,'b',t1,ones(1,length(t1))*omega,'--');
p1(2).Color='k';
l1=legend(p1,{'$\omega_R(t)$','$\omega_{R,max}$'},'Interpreter','latex');
l1.set('FontSize',15)
grid;
xlbl1=xlabel('$t[sec]$','Interpreter','latex');
ylbl1=ylabel('$\omega_R(t)[RAD/sec]$','Interpreter','latex');
ylbl1.FontSize=13;
xlbl1.FontSize=13;
axis([0 25.8 6 10.5])

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
axis([0 25.8 0 10.5])
% 
% figure(4)
% p=plot(t1,theta,'b',[0:Ts:],thetarr,'r--');
% title('$\theta-Robot$, versus $\theta-Ref$','Interpreter','latex');
% p(1).LineWidth = 1.25;
% p(2).LineWidth = 0.9;
% l1=legend(p,{'$\theta Robot$','$\theta Ref$'},'Interpreter','latex');
% l1.set('FontSize',15)
% xlbl1=xlabel('$t[sec]$','Interpreter','latex');
% ylbl1=ylabel('$\theta [RAD]$','Interpreter','latex');
% ylbl1.FontSize=13;
% xlbl1.FontSize=13;
% axis([0 39 -2 6])
% 
figure(5)
p=plot(t1,trackerr,'b');
title('Tracking error','Interpreter','latex')
p(1).LineWidth = 1;
l1=legend(p,{'track err'},'Interpreter','latex');
l1.set('FontSize',15)
xlbl1=xlabel('$t[sec]$','Interpreter','latex');
ylbl1=ylabel('$Err [m]$','Interpreter','latex');
ylbl1.FontSize=13;
xlbl1.FontSize=13;
axis([0 25.8 -0.05 0.6])
