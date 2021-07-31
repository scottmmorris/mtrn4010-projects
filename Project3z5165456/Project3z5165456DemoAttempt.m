
% ---------------------------------------------------------------------
% MTRN4010/2021.T1

% Authored by Jose Guivant
% Edited by Scott Morris z5165456

% Script to visualise UGV LiDAR data for Project 2 in MTRN4010
function Project3z5165456DemoAttempt()

    % Define global variables to handle the plotting of Local (lhand) and Global
    % coordinates (ghand)
    global ghand ghand2 ghand3 ghand4 ghand5 ghand6 ghand7 ghand8 ghand9 ghand10 ghand11 ghand12 ghand13 ghand14 Landmarks;

    global stdDevAngRate stdDevSpeed stdDevRange stdDevBearing
    
    % Load the UGV mesurements (time, speed, angular rate)
    Data=load('Measurements_AAS01.mat');
    Data=Data.A; 
    
    % Load the landmarks cartesian coordinates
    Landmarks = load('MapMTRN4010.mat');
    Landmarks = Landmarks.Map;
    
    % Create a plot for the Cartesian Local LiDAR scans 
    figure(4) ; clf(); zoom on; grid on; hold on;
    title('UGV LiDAR scans (in cartesian, global frame)');
    ylabel('y (m)'); xlabel('x (m)'); axis([-5 5 0 8]);
    
    % Create a handle to set the LiDAR points, brilliant points, OOIs and
    % UGV position
    ghand = plot(0,0,'.b');
    ghand2 = plot(0,0,'.r');
    ghand3 = plot(0, 0, 'sr');
    ghand4 = plot(0,0,'gx');
    ghand5 = plot(0,0,'m');
    ghand6 = plot(0,0,'m');
    ghand7 = plot(0,0,'m');
    ghand8 = plot(0,0,'m');
    ghand9 = plot(0,0,'m');
    ghand10 = plot(0,0,'go');
    ghand11 = plot(0,0,'co');
    ghand12 = plot(0,0,'mo');
    ghand13 = plot(0,0,'ko');
    ghand14 = plot(0,0,'yo');
    
    % Plot the landmarks 
    plot(Landmarks.x, Landmarks.y, 'rs');
    legend({'points', 'brilliant points', 'Landmarks', 'LiDAR', 'Association 1', 'Association 2', 'Association 3', 'Association 4', 'Association 5', 'Landmark 1', 'Landmark 2', 'Landmark 3', 'Landmark 4', 'Landmark 5', 'Landmarks'}, 'Location', 'bestoutside');
    
    % Define the std deviation of some measurements
    stdDevAngRate = 1.7*pi/180;
    stdDevSpeed = 0.3;
    stdDevRange = 0.35;
    stdDevBearing = 2.5*pi/180;
    
    % Initialise expectation and covariance of state variable X
    P = zeros(3,3);
    
    % Initialise covariance of model inputs
    Q = diag( [ (0.01)^2 ,(0.01)^2 , (1*pi/180)^2]) ;
    Q_u = diag([stdDevAngRate stdDevSpeed]);
    
    % Get number of samples in the dataset
    L=Data.L;

    % Initialise dynamic parameters (X(1) = x position, X(2) = y position,
    % X(3) = heading)
    XeH = zeros(3, L);
    X = [0; 0; pi/2];
    XeH(:,1) = [0; 0; pi/2];
    Xe = XeH(:, 1);
    % Set the initial time in seconds
    i0 = 1;
    t0=0.0001*double(Data.t(i0));
    
    % Finding the sample at 25 seconds
    while Data.t(i0) < 100000 + 25000
        i0 = i0 + 1;
    end
    
    % Calculate bias (average) of the first 25 seconds
    bias = 0.01 * mean(Data.Z(2, 1:i0));
    
    % Re-initialise time samples
    i0 = 1;
    
    % Loop through all the data scans
    for i=i0:L
        % Import the LiDAR data at the current time
        m = Data.Z(:,i);
        
        % Obtain whether there is a LiDAR scan at this sample
        indexScan = m(3);

        % Process the LiDAR data only if there is a scan at this time
        if (indexScan>1)
            
            % Get the current time sample
            t=0.0001*double(Data.t(i));

            % Calculate the new speed (m/s) and the angular rate change
            % (deg/sec) of the current data. Subtract the bias from the angular
            % rate
            NewSpeed = double(m(1))*0.001;
            NewAngRate = double(m(2))*0.01 - bias; 

            % Calculate the change in time between last sample and current
            % sample
            dt = t-t0;
            t0=t;

            Xe = Xe + dt * [NewSpeed * cos(Xe(3));
                NewSpeed * sin(Xe(3));
                deg2rad(NewAngRate)];
            
            X = X + dt * [NewSpeed * cos(X(3));
            NewSpeed * sin(X(3));
            deg2rad(NewAngRate)];
            
                                
            set(ghand4, 'xdata', Xe(1), 'ydata', Xe(2));
            % Calculate the Jacobian matrix of the simulated model at the
            % current time
            J = [ [1, 0, -dt * NewSpeed * sin(Xe(3))];
                  [0, 1, dt * NewSpeed * cos(Xe(3))];
                  [0, 0, 1]];

            % Calculate the Q1 matrix by first calculating the Jacobian of the
            % inputs
            Ju = [ [dt * cos(Xe(3)), 0];
                   [dt * sin(Xe(3)), 0];
                   [0, dt] ];

            % Update the covariance matrix
            P = J * P * J' + (Q + Ju * Q_u * Ju');
            
            % Extract the ranges and intensities, of the LiDAR data
            [r,I]=GetRangeAndIntensityFromRawScan(Data.scans(:, indexScan ));
            
            ii=find(I>0);
            
            % Process the LiDAR data given the range, intensity and current
            % state vector
            Xe
            X
            [xg, yg] = ProcessLiDAR( r, Xe );
            yg(180)
            set(ghand,'xdata', xg, 'ydata', yg);
            set(ghand2,'xdata', xg(ii),'ydata', yg(ii));
            
            ooi = getOOIPositions( r, I, xg, yg );
            
            set(ghand3, 'xdata', ooi(:, 1), 'ydata', ooi(:, 2));
            
            connOOI = associateLandmarks(Landmarks, ooi(:, 1), ooi(:, 2));
            
            % Define OOI association
            updateDataAssociation([ghand10, ghand11, ghand12, ghand13, ghand14], connOOI);
            
            % Loop through each detected landmark
            for c = 1:length(connOOI)
                % Check if a landmark was actually detected
                if (connOOI(c,1) ~= 0 && connOOI(c,2) ~= 0)
                    % Calculate the expected distance to the landmark
                    eDX = (Landmarks.x(c));      % (xu-x)
                    eDY = (Landmarks.y(c));      % (yu-y)
                    eDD = sqrt( eDX*eDX + eDY*eDY );
                    
                    % Calculate the Jacobian of the observation functions
                    H = [[-eDX/eDD, -eDY/eDD, 0 ]; ...
                        [eDY/(eDX^2 + eDY^2), -eDX/(eDX^2 + eDY^2), -1]];
                    
                    ExpectedRange = eDD;
                    ExpectedBearing = atan2(eDY, eDX) - Xe(3) + pi/2;
                    % Calculate the residuals of the range and bearing
                    z  = [[sqrt(connOOI(c, 1)^2 + connOOI(c, 2)^2) - ExpectedRange]; 
                          [atan2(connOOI(c, 2), connOOI(c, 1)) - Xe(3) + pi/2 - ExpectedBearing]];
                    
                    % Calculate the uncertainty covariance in measurements
                    R = diag([stdDevRange*stdDevRange*4 stdDevBearing*stdDevBearing*4]);
                    
                    % EKF update with measured ranges & bearings
                    [Xe, P] = DoUpdate(P, Xe, H, R, z);

                end
            end
            
            % Introduce a short pause in order to visualise scans
            pause(0.01); 
        end
        XeH(:, i) = Xe;
    end
    
    % Plot the overall path of the UGV estimated by the kinematic model
    figure(5); hold on; grid on;
    title('UGV Path');
    xlabel('x(m)');
    ylabel('y(m)');
    axis([-1.5 3.5 -1 5]);
    plot(XeH(1, :), XeH(2, :), 'g');
    
    % Plot the overall heading of the UGV estimated by the kinematic model
    figure(6); hold on; grid on;
    title('Heading');
    axis([0 250 -350 100]);
    plot(0.0001*double(Data.t), rad2deg(XeH(3, :)), 'g');
end 

% -------------------------------------------------------

% Function for parsing the binary scan; extracting ranges and intensities of reflections
function [r,I] = GetRangeAndIntensityFromRawScan(scan)
    % Bits [0:12] are range, bits [13:15] intensity
    r = 0.01*single(bitand(scan,8191));
    I= bitshift(scan,-13);      
end

% -------------------------------------------------------

% Function for processing and plotting the local and global cartesian coordinates of the UGV
function [xg, yg] = ProcessLiDAR( r , X )
    % Define global handles for plots
    X(2)
    % Convert the polar coordinates of the LiDAR into cartesian
    % coordinates using trigonometry and set these on the plot
    ang = 0:0.5:180;
    x = transpose(r) .* cos(deg2rad(ang));
    y = transpose(r) .* sin(deg2rad(ang));
    
    % Convert the local coordinates into global using the LiDAR
    % displacement, a rotation transformation and linear transformation
    xc = x;
    yc = y + 0.46;
    xg = xc * cos(X(3) - pi/2) - yc * sin(X(3) - pi/2) + X(1);
    yg = xc * sin(X(3) - pi/2) + yc * cos(X(3) - pi/2) + X(2);
end

function ooi = getOOIPositions(r, I, x, y)
    % Used to calculate the postion of the OOIs
    ooi = zeros(1, 2);
    n = 1; m = 1;
    
    ii = find(I>0);
    
    % Check that the end of the LiDAR scan has not been reached
    while n < length(r(ii)) - 1
        bp = 0;
        counter = 0;
        w_x = 0;
        w_y = 0;
        brillx = x(ii);
        brilly = y(ii);
        % Check if the distance between the current and next points range
        % is less than 0.1m for clusters of points
        while (sqrt((brillx(n) - brillx(n + 1))^2 + (brilly(n) - brilly(n + 1))^2) < 0.6 && n < length(r(ii)) - 1)
            % If the intensity of one of these points in the cluster is
            % greater than 1, make it an OOI
            bp = 1;
            % add the x and y coordinates of each point in the cluster
            w_x = w_x + x(ii(n));
            w_y = w_y + y(ii(n));
            n = n + 1;
            counter = counter + 1;
        end
        % If a brilliant point was detected add the next brilliant point
        if (bp == 1)
            w_x = w_x + x(ii(n));
            w_y = w_y + y(ii(n));
            counter = counter + 1;
        end
        % If the cluster is an OOI
        if(bp == 1)
            % Calculae the average x and y position of all the points in
            % the cluster
            ooi(m, 1) = w_x/counter;
            ooi(m, 2) = w_y/counter;
            m = m + 1;
        end
        n = n + 1;
    end
end

function connOOI = associateLandmarks(Landmarks, ooix, ooiy)
    % Initialise data association matrix
    connOOI = zeros(Landmarks.n, 2);
    
    % For all the landmarks provided
    for z = 1:Landmarks.n
        % For all OOIs detected
        for b = 1:length(ooix)
            % Check the distance between them is 0.6m
            if(sqrt(((Landmarks.x(z) - ooix(b))^2 + ((Landmarks.y(z) - ooiy(b))^2))) < 0.6)
                % If it is store the Cartesian coordinates in the
                % associated landmarks matrix
                connOOI(z, 1) = ooix(b);
                connOOI(z, 2) = ooiy(b);
                break;
            end
        end
    end
end

function updateDataAssociation(handles, connOOI)
        for f=1:5
            if(connOOI(f,1) ~= 0 && connOOI(f,2) ~= 0)
                set(handles(f), 'xdata', connOOI(f,1), 'ydata', connOOI(f,2))
            else
                set(handles(f), 'xdata', -999999, 'ydata', -999999);
            end
        end
end

% ---------------------------------------------------------------------

% Function for performing EKF update
function [Xe,P]=DoUpdate(P,Xe,H,R,z)
    % Some intermediate steps for the EKF (as presented in the lecture notes)
    S = R + H * P * H';
    iS = inv(S);                 % iS = inv(S) ;   % in this case S is 1x1 so inv(S) is just 1/S
    K = P*H'*iS;           % Kalman gain
    % ----- finally, we do it...We obtain  X(k+1|k+1) and P(k+1|k+1)
    Xe = Xe + K * z;       % update the  expected value
    P = P - K * H * P;       % update the Covariance % i.e. "P = P-P*H'*iS*H*P"  )
            
    return
end
% ---------------------------------------------------------------------