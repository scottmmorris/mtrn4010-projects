% MTRN4010 T1 2021
% Authored by Scott Morris z5165456

function xyE=GetMySolutionXY()

    % Obtain the input data
    global Data5;
    
    % Output the landmark ranges
    disp('testing: being called with these values in Data5.ranges:'); disp('ranges'); disp(Data5.ranges');
    
    % Set the initial search conditions
    xy0 = [0, 0];
    
    % Minimise the cost function using the initial position
    xyE = fminsearch(@CostFunctionRangeObservations,xy0);
   
end

% Implementation of cost function for observed ranges
function xy = CostFunctionRangeObservations(x)

    % Obtain the input data
    global Data5;

    % Initialise the sum
    xy = 0;
    
    % Loop through provided landmarks (5)
    for k = 1:5
        % Apply range equation for the specified landmark
        xy = xy + abs(Data5.ranges(k) - sqrt((x(1) - Data5.Lx(k))^2 + (x(2) - Data5.Ly(k))^2));
    end
    
end

