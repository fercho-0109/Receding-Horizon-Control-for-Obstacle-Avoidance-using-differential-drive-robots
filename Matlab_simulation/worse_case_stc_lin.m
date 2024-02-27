function [Q0,Y0]=worse_case_stc_lin(Err0,A,B,ru,Qbd,Qx,Qu)
n=size(A,1);
m=size(B,2);

Q=sdpvar(n,n); % create the unknow variables
gamma=sdpvar(1,1); 
Y = sdpvar(m,n); 
% LMI constrains
F1=([1 Err0';
    Err0 Q ]>=0);

F2=([Q         Q*A'+Y'*B' Q*sqrt(Qx)    Y'*sqrt(Qu)
    A*Q+B*Y        Q      zeros(2)       zeros(2)
    sqrt(Qx)*Q  zeros(2)  gamma*eye(2)   zeros(2)
    sqrt(Qu)*Y  zeros(2)  zeros(2)      gamma*eye(2)]>=0);

F3=([Q]>=0);

F4=([ru*ru*eye(n) Y;
    Y' Q]>=0);

F5=(Q==Qbd);

F=F1+F2+F3+F4+F5;

% solution
opts=sdpsettings('solver','mosek','verbose',0);
solvesdp(F,gamma,opts);

Q0=double(Q);
Y0=double(Y);
end