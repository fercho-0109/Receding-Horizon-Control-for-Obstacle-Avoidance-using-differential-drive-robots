function [Q_sol,Y_sol,time] = dyn_FL_kothare_constraint_OA(Ad,Bd,x0,Q1,R,H,Q0)

n=size(Ad,1);
m=size(Bd,2);


%Qb=(d^2*eye(2));


%c=[1 0 0 0; 0 1 0 0 ];

setlmis([])
Q = lmivar(1,[n 1]);
Qj=lmivar(1,[n 1]);
Y = lmivar(2,[m n]);
gamma=lmivar(1,[1 1]);

%LMI constraints
lmiterm([-1 1 1 Q],1,1);%constraint 1

lmiterm([-2,1,1,Q],1,1);  %constraint 2 block (1,1)
lmiterm([-2,1,1,-Q],1,1);  %constraint 2 block (1,1)
lmiterm([-2,1,1,Qj],-1,1);  %constraint 2 block (1,1)
lmiterm([-2,2,1,Q],Ad,1); %constraint 2 block (2,1)
lmiterm([-2,2,1,Y],Bd,1); %constraint 2 block (2,1)
lmiterm([-2,2,2,Q],1,1);  %constraint 2 block (2,2)
lmiterm([-2,3,1,Q],sqrtm(Q1),1); %constraint 2 block (3,1)
lmiterm([-2,3,2,0],zeros(n,n));  %constraint 2 block (3,2)
lmiterm([-2,3,3,gamma],1,eye(n));%constraint 2 block (3,3)
lmiterm([-2,4,1,Y],sqrtm(R),1);  %constraint 2 block (4,1)
lmiterm([-2,4,2,0],zeros(m,n));  %constraint 2 block (4,2)
lmiterm([-2,4,3,0],zeros(m,n));  %constraint 2 block (4,3)
lmiterm([-2,4,4,gamma],1,eye(m));%constraint 2 block (4,4)

lmiterm([-3,1,1,0],1); %constraint 3 block (1,1)
lmiterm([-3,2,1,0],x0); %constraint 3 block (2,1)
lmiterm([-3,2,2,Q],1,1);%constraint 3 block (2,2)

lmiterm([-4,1,1,Q],1,1); %constraint 4 block (1,1)
lmiterm([-4,2,1,Y],H(1,:),1); %constraint 4 block (2,1)
lmiterm([-4,2,2,0],1); %constraint 4 block (2,2)

lmiterm([-5,1,1,Q],1,1); %constraint 5 block (1,1)
lmiterm([-5,2,1,Y],H(2,:),1); %constraint 5 block (2,1)
lmiterm([-5,2,2,0],1); %constraint 5 block (2,2)

lmiterm([-6,1,1,Q],1,1); %constraint 6 block (1,1)
lmiterm([-6,2,1,Y],H(3,:),1); %constraint 6 block (2,1)
lmiterm([-6,2,2,0],1); %constraint 6 block (2,2)

lmiterm([-7,1,1,Q],1,1); %constraint 7 block (1,1)
lmiterm([-7,2,1,Y],H(4,:),1); %constraint 7 block (2,1)
lmiterm([-7,2,2,0],1); %constraint 7 block (2,2)

lmiterm([-8 1 1 Qj],1,1);%constraint 1

lmiterm([9 1 1 Q],1,1);%constraint 1
lmiterm([-9 1 1 0],(Q0));%constraint 1
% lmiterm([9 1 1 Q],1,1);%constraint 1
% lmiterm([-9 1 1 0],Qbz);%constraint 1

lmis = getlmis;

% c = defcx(lmis,8,Q,Y,gamma)

c=zeros(11,1);
c(11)=1;
options = [1e-4,0,0,0,1];
[time,sol]=mincx(lmis,c,options);


Q_sol = dec2mat(lmis,sol,Q);
Y_sol = dec2mat(lmis,sol,Y);

F_sol=Y_sol/Q_sol;

end

