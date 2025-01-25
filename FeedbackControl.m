function [twist, Vel, X_error, cum_error] = FeedbackControl(state, X, X_d, X_d_next, Kp, Ki, dt,cum_error)
X_error = se3ToVec(logm(X\X_d));
Vd = se3ToVec((1/dt)*logm(X_d\X_d_next));
ad = Adjoint(X\X_d);




V = ad*Vd + Kp*X_error + Ki*(cum_error);
% V = Kp*X_error + Ki*(cum_error);

twist = V;

r = 0.0475;
l = 0.235;
w = 0.15;

F = (r/4)*[ -1/(l + w),  1/(l + w),  1/(l + w), -1/(l + w);
           1,          1,          1,         1;
          -1,          1,         -1,         1];

F6 = [0,0,0,0;0,0,0,0;F;0,0,0,0];

B1=  [0,0,1,0,0.033,0];
B2 = [0,-1,0,-0.5076,0,0];
B3 = [0,-1,0,-0.3526,0,0];
B4 = [0,-1,0,-0.2176,0,0];
B5 = [0,0,1,0,0,0];
Blist =[B1',B2',B3',B4',B5'];

M = [1,0,0,0.033;0,1,0,0;0,0,1,0.6546;0,0,0,1];

J_arm = JacobianBody(Blist,state(4:8)');

% fixed offset from {b} to base of arm {0}
Tb0 = [[1, 0, 0, 0.1662];
        [0, 1, 0, 0];
        [0, 0, 1, 0.0026];
        [0, 0, 0, 1]];

T0e = FKinBody(M, Blist, state(4:8)');

J_base = Adjoint(pinv(T0e, 1e-3)*pinv(Tb0, 1e-3))*F6;

J_e = [J_base,J_arm];

J_e_inv = pinv(J_e,1e-3);
Vel = J_e_inv*V;

end
