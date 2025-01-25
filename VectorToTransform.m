function T = VectorToTransform(vector)
    % Extract rotation matrix components
    R = [vector(1), vector(2), vector(3);
         vector(4), vector(5), vector(6);
         vector(7), vector(8), vector(9)];
    
    % Extract position components
    p = [vector(10); vector(11); vector(12)];
    
    % Construct the transformation matrix
    T = [R, p; 0,0,0,1];
end