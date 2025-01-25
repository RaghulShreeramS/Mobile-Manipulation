function [trajectory, csv_file] = Trajectory_Generator(Tse_initial, Tsc_initial, Tsc_final, Tce_grasp, Tce_standoff, k)
    % Define the time step and total time for trajectory segments
    dt = 0.01;
    totalTime = 2;  % total time for each segment
    totalTime_gripper = 1;

    % Calculate the number of points in each trajectory segment
    N = totalTime / dt * k;
    N_gripper = totalTime_gripper/dt*k;
    % N = 10;

    % Define the configurations for the trajectory
    Tse_standoff_initial = Tsc_initial * Tce_standoff;
    Tse_grasp = Tsc_initial * Tce_grasp;
    Tse_standoff_final = Tsc_final*Tce_standoff;

    Tse_grasp_final = Tsc_final * Tce_grasp;

    % Generate the trajectory segments
    segment1 = ScrewTrajectory(Tse_initial, Tse_standoff_initial, totalTime, N, k);
    segment2 = ScrewTrajectory(Tse_standoff_initial, Tse_grasp, totalTime, N, k);
    segment3 = ScrewTrajectory(Tse_grasp, Tse_grasp, totalTime_gripper, N_gripper, k); % Assume instantaneous action for gripper close
    segment4 = ScrewTrajectory(Tse_grasp, Tse_standoff_initial, totalTime, N, k);
    segment5 = ScrewTrajectory(Tse_standoff_initial, Tse_standoff_final, totalTime, N, k);
    segment6 = ScrewTrajectory(Tse_standoff_final, Tse_grasp_final, totalTime, N, k);
    segment7 = ScrewTrajectory(Tse_grasp_final, Tse_grasp_final, totalTime_gripper, N_gripper, k); % Assume instantaneous action for gripper open
    segment8 = ScrewTrajectory(Tse_grasp_final, Tse_standoff_final, totalTime, N, k);

    % Combine all segments into a single trajectory
    segments = {segment1, segment2, segment3, segment4, segment5, segment6, segment7, segment8};
    disp(segments)
    % Initialize the trajectory matrix
    trajectory = [];
    gripper_state = 0;  % Default state (open)
    % Process each segment and append it to the trajectory
    for i = 1:length(segments)
        segment = segments{i};
        for j = 1:size(segment,2) 
            T = segment{j};
            row = [T(1,1),T(1,2),T(1,3),T(2,1),T(2,2),T(2,3),T(3,1),T(3,2),T(3,3),T(1,4),T(2,4),T(3,4)];

            % Determine the gripper state based on the segment
            if i == 3  % Gripper closes in the third segment
                gripper_state = 1;
            elseif i == 7  % Gripper opens in the seventh segment
                gripper_state = 0;
            end

            trajectory = [trajectory; [row, gripper_state]];
        end
    end

    % Write the trajectory to a CSV file
    csv_file = 'trajectory_array.csv';
    writematrix(trajectory,csv_file);
end