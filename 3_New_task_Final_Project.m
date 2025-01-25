clear;
%% KP and KI values update
Kp = 1.5*eye(6);
Ki = 0.001* eye(6);


%% Default configurations
dt = 0.01;

Tb0 = [1, 0, 0, 0.1662;
        0, 1, 0, 0;
        0, 0, 1, 0.0026;
        0, 0, 0, 1];

M = [1,0,0,0.033;
    0,1,0,0;
    0,0,1,0.6546;
    0,0,0,1];


max_speed = 60;

B1=  [0,0,1,0,0.033,0];
B2 = [0,-1,0,-0.5076,0,0];
B3 = [0,-1,0,-0.3526,0,0];
B4 = [0,-1,0,-0.2176,0,0];
B5 = [0,0,1,0,0,0];
Blist =[B1',B2',B3',B4',B5'];

%% Initial State
 
% Initial state for you_bot 
state = [-0.2 0 -1 0.3 -0.15 -1.9 -0.15 0.15 0 0 0 0 0];

% Initial state for the reference Trajectory
state_ref = [-0.2 -0 -1.2 0.8 -0.15 -1.9 -0.15 0.15 0 0 0 0 0];

% Initial error of the e-e (state_ref) is more than 30 degrees and chassis is more than
% 0.2m


%Calculating the state matrix of e-e from the state vector
phi = state_ref(1);
x_b = state_ref(2);
y_b = state_ref(3);

Tsb = [cos(phi), -sin(phi), 0, x_b;
        sin(phi), cos(phi),  0  y_b;
        0,        0,         1, 0.0963;
        0,        0,         0, 1];

T0e = FKinBody(M, Blist, state_ref(4:8)');

X = Tsb*Tb0*T0e; % initial End-effector configuration for reference Trajectory 



Tsc_initial = [1, 0, 0, 0.5;
              0, 1, 0, 0;
              0, 0, 1, 0.025;
              0, 0, 0, 1];

Tsc_final = [0, 1, 0, 1;
            -1, 0, 0, -0.5;
            0, 0, 1, 0.025;
            0, 0, 0, 1];


Tce_grasp = [cos(3*pi/4), 0, sin(3*pi/4), 0;
                0, 1, 0, 0;
                -sin(3*pi/4), 0, cos(3*pi/4), 0;
                0, 0, 0, 1];

Tce_standoff = [cos(3*pi/4), 0, sin(3*pi/4), 0;
                0, 1, 0, 0;
                -sin(3*pi/4), 0, cos(3*pi/4), 0.2;
                0, 0, 0, 1];


%% Calling the Trajectory Generated to get the Reference Trajectory ------------------------------------------------------------------
k = 3;
[ref_trajectory, csv_file] = Trajectory_Generator(X, Tsc_initial, Tsc_final, Tce_grasp, Tce_standoff, k);
[numRows, ~] = size(ref_trajectory);

%% Control Law ------------------------------------------------------------------------------------------------------------------------
Wrapper_output = []; %Intializing array for storage of the states to save in csv file

X_ERROR = [];
cum_error = zeros(6,1); % Intializing the cumulative error for Ki control

for i = 1:numRows-1

    traj_d = ref_trajectory(i,:); % Desired state
    traj_d_next = ref_trajectory(i+1,:); % Desired next state

    Xd = VectorToTransform(traj_d(1:12));           % Converting the vector to Tmatrix - 
    Xd_next = VectorToTransform(traj_d_next(1:12)); % for desired and desired next states

    % Feedback Control law is implemented here
    [Twist,Vel, X_error, cum_error] = FeedbackControl(state, X, Xd, Xd_next, Kp, Ki, dt, cum_error); 
    X_ERROR = [X_ERROR,X_error]; % X_error is stored here


    controls = [Vel(5:9)', Vel(1:4)']; % controls is returned from the Feedback_back control
    cum_error = cum_error + X_error*dt; %Cummulative error updated here


    % State is updated here by calling the Next_State function
    state = NextState(state, controls, dt, max_speed);

    %Recalculating the X using the updated next from Next state vector
    phi = state(1);
    x_b = state(2);
    y_b = state(3);
    Tsb = [cos(phi), -sin(phi), 0, x_b;
           sin(phi), cos(phi),  0, y_b;
           0,        0,         1, 0.0963;
           0,        0,         0, 1];
    T0e = FKinBody(M, Blist, state(4:8)');
    X = Tsb * Tb0 * T0e;  % Recalculate X

    % Updated state after Feedback_Control is stored here
    Wrapper_output = [Wrapper_output; [state,ref_trajectory(i,13)]];
    
end

%Storing all the states as Final_trajectory here - Wrapped Trajectory
writematrix(Wrapper_output, "Wrapper_output_New_Task.csv")

figure;
hold on;  % all plots are on the same figure
time = (1:numRows-1) * dt/k;  % Create a time vector 
%% PLotting X_error graph
for i = 1:6
    plot(time, X_ERROR(i, :));
end

title('New Task');
xlabel('Time (seconds)');
ylabel('Error Components');
legend('X_err1', 'X_err2', 'X_err3', 'X_err4', 'X_err5', 'X_err6');
hold off;

