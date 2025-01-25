function next_state = NextState(curr_state, velocities, dt, v_max)

next_state = zeros(1,12);

for i = 1:9
    if velocities(i)>0 && velocities(i) > v_max
        next_state(i+3) = curr_state(i+3) + v_max*dt;
    elseif velocities(i)<0 && velocities(i) < -v_max
        next_state(i+3) = curr_state(i+3) + -v_max*dt;
    else
        next_state(i+3) = curr_state(i+3) + velocities(i)*dt;
    end
end

r = 0.0475;
l = 0.235;
w = 0.15;

F = (r/4)*[ -1/(l + w),  1/(l + w),  1/(l + w), -1/(l + w);
           1,          1,          1,         1;
          -1,          1,         -1,         1];

Vb = F*dt*velocities(6:9)';
% Tbb = expm(VecTose3([0;0;Vb(1);Vb(2);Vb(3);0]));

omega_bz = Vb(1);
vbx = Vb(2);
vby = Vb(3);

if omega_bz == 0
    % Case when omega_bz equals 0
    Delta_qb = [0; vbx; vby];
else
    % Case when omega_bz does not equal 0
    Delta_qb = [omega_bz;
                (vbx * sin(omega_bz) + vby * (cos(omega_bz) - 1)) / omega_bz;
                (vby * sin(omega_bz) + vbx * (1 - cos(omega_bz))) / omega_bz];
end

phi_k = curr_state(1);

% Define the transformation matrix
Tsb = [1,          0,           0;
       0, cos(phi_k), -sin(phi_k);
       0, sin(phi_k),  cos(phi_k)];

% Calculate the resulting vector Delta_q
Delta_q = Tsb * Delta_qb;


next_state(1:3) = curr_state(1:3) + Delta_q';

