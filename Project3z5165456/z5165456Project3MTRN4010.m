
% ---------------------------------------------------------------------
% MTRN4010/2021.T1

% Authored by Jose Guivant
% Edited by Scott Morris z5165456

% Script to visualise UGV LiDAR data for Project 2 in MTRN4010
function z5165456Project3MTRN4010()

    % Define global variables to handle the plotting of Local (lhand) and Global
    % coordinates (ghand)
    global lhand lhand2 lhand3 ghand ghand2 ghand4 ghand5 ghand6 ghand7 ghand8 ghand9 ghand10 ghand11 ghand12 ghand13 ghand14 ehand ehand2 ehand4 ehand5 ehand6 ehand7 ehand8 ehand9 ehand10 ehand11 ehand12 ehand13 ehand14 Landmarks;

    global stdDevAngRate stdDevSpeed stdDevRange stdDevBearing
    
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
    
    % Load the UGV mesurements (time, speed, angular rate)
    Data=load('Measurements_AAS01.mat');
    Data=Data.A; 
    
    % Load the landmarks cartesian coordinates
    Landmarks = load('MapMTRN4010.mat');
    Landmarks = Landmarks.Map;

    % Create a variable to store the LiDAR angle increments
    angleScan = [0:360]/2;
    
    % Load the range data into a variable r
    [r,~]=GetRangeAndIntensityFromRawScan(Data.scans(:,1));
    
    % Create a plot for the Polar Local LiDAR scans 
    figure(1) ; clf(); hold on; hL=plot(angleScan,r,'.'); zoom on; grid on;
    title('UGV LiDAR scans (in polar, local frame)');
    ylabel('range (m)'); xlabel('angle (degrees)'); axis([0,180,0,20]); 

    % Create a handle to set the brilliant points
    hL2=plot(0,0,'.r');
    legend({'points','brilliant points'});
    
    % Create a plot for the Cartesian Local LiDAR scans 
    figure(2) ; clf(); hold on; zoom on; grid on;
    title('UGV LiDAR scans (in cartesian, local frame');
    ylabel('y (m)'); xlabel('x (m)'); axis([-7 7 -3 11]); 

    % Create a handle to set the LiDAR points, brilliant points and OOIs
    lhand=plot(0,0,'.b');
    lhand2=plot(0,0,'.r');
    lhand3=plot(0,0,'or');
    
    % Plot the UGV position at (0, 0)
    plot(0,0,'gx');
    legend({'points', 'brilliant points', 'OOI', 'LiDAR'});
    
    % Create a plot for the Cartesian Local LiDAR scans 
    figure(4) ; clf(); zoom on; grid on; hold on;
    title('UGV LiDAR scans (in cartesian, global frame)');
    ylabel('y (m)'); xlabel('x (m)'); axis([-5 5 0 8]);
    
    % Create a handle to set the LiDAR points, brilliant points, OOIs and
    % UGV position
    ghand = plot(0,0,'.b');
    ghand2 = plot(0,0,'.r');
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
    legend({'points', 'brilliant points', 'LiDAR', 'Association 1', 'Association 2', 'Association 3', 'Association 4', 'Association 5', 'Landmark 1', 'Landmark 2', 'Landmark 3', 'Landmark 4', 'Landmark 5', 'Landmarks'}, 'Location', 'bestoutside');
    zoom on;
    
        % Create a plot for the Cartesian Local LiDAR scans 
    figure(3) ; clf(); zoom on; grid on; hold on;
    title('UGV LiDAR scans (in cartesian, global frame) with EKF');
    ylabel('y (m)'); xlabel('x (m)'); axis([-5 5 0 8]);
    
    % Create a handle to set the LiDAR points, brilliant points, OOIs and
    % UGV position
    ehand = plot(0,0,'.b');
    ehand2 = plot(0,0,'.r');
    ehand4 = plot(0,0,'gx');
    ehand5 = plot(0,0,'m');
    ehand6 = plot(0,0,'m');
    ehand7 = plot(0,0,'m');
    ehand8 = plot(0,0,'m');
    ehand9 = plot(0,0,'m');
    ehand10 = plot(0,0,'go');
    ehand11 = plot(0,0,'co');
    ehand12 = plot(0,0,'mo');
    ehand13 = plot(0,0,'ko');
    ehand14 = plot(0,0,'yo');
    
    % Plot the landmarks 
    plot(Landmarks.x, Landmarks.y, 'rs');
    legend({'points', 'brilliant points', 'LiDAR', 'Association 1', 'Association 2', 'Association 3', 'Association 4', 'Association 5', 'Landmark 1', 'Landmark 2', 'Landmark 3', 'Landmark 4', 'Landmark 5', 'Landmarks'}, 'Location', 'bestoutside');
    zoom on;

    % Get number of samples in the dataset
    L=Data.L;

    % Initialise dynamic parameters (X(1) = x position, X(2) = y position,
    % X(3) = heading)
    X = zeros(3, L);
    X(:,1) = [0; 0; pi/2];
    XeH = zeros(3, L);
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
    
    T0=0.0001*double(Data.t(i0));

    % Loop through all the data scans
    for i=i0:L
        % Get the current time sample
        t=0.0001*double(Data.t(i));
        
        % Import the LiDAR data at the current time
        m = Data.Z(:,i);

        % Calculate the new speed (m/s) and the angular rate change
        % (deg/sec) of the current data. Subtract the bias from the angular
        % rate
        NewSpeed = double(m(1))*0.001;
        NewAngRate = double(m(2))*0.01 - bias; 
        
        % Obtain whether there is a LiDAR scan at this sample
        indexScan = m(3);  
        
        % Calculate the change in time between last sample and current
        % sample
        dt = t-t0; t0=t;
        
        % Use the kinematic model to estimate the UGVs pose (X(1, 2)) and
        % heading (X(3))
        X(:, i+1) = X(:, i) + dt * [NewSpeed * cos(X(3, i));
                                    NewSpeed * sin(X(3, i));
                                    deg2rad(NewAngRate)];
        % Use the kinematic model to update the expected state variable
        Xe = Xe + dt * [NewSpeed * cos(Xe(3));
                                    NewSpeed * sin(Xe(3));
                                    deg2rad(NewAngRate)];
        % Process the LiDAR data only if there is a scan at this time
        if (indexScan>1)
            
            Dt = t - T0;
            % Calculate the Jacobian matrix of the simulated model at the
            % current time
            J = [ [1, 0, -Dt * NewSpeed * sin(Xe(3))]; [0, 1, Dt * NewSpeed * cos(Xe(3))]; [0, 0, 1]];

            % Calculate the Q1 matrix by first calculating the Jacobian of the
            % inputs
            Ju = Dt * [[NewSpeed * cos(Xe(3)), 0]; [NewSpeed * sin(Xe(3)), 0]; [0, 1]];

            % Update the covariance matrix
            P = J * P * J' + (Q + Ju * Q_u * Ju');
            
            % Extract the ranges and intensities, of the LiDAR data
            [r,I]=GetRangeAndIntensityFromRawScan(Data.scans(:, indexScan ));
            
            % Process the LiDAR data given the range, intensity and current
            % state vector
            conOOI = ProcessLiDAR( r, I , Xe );
            
            % Loop through each detected landmark
            for c = 1:length(conOOI)
                % Check if a landmark was actually detected
                if (conOOI(c,1) ~= 0 && conOOI(c,2) ~= 0)
                    % Calculate the expected distance to the landmark
                    eDX = (Landmarks.x(c)-Xe(1));      % (xu-x)
                    eDY = (Landmarks.y(c)-Xe(2));      % (yu-y)
                    eDD = sqrt( eDX*eDX + eDY*eDY );
                    
                    % Calculate the Jacobian of the observation functions
                    H = [[-eDX/eDD, -eDY/eDD, 0 ]; ...
                        [eDY/(eDX^2 + eDY^2), -eDX/(eDX^2 + eDY^2), -1]];
                    
                    ExpectedRange = eDD
                    sqrt(conOOI(c, 1)^2 + conOOI(c, 2)^2)
                    ExpectedBearing = atan2(eDY, eDX) - Xe(3) + pi/2
                    atan2(conOOI(c, 2), conOOI(c, 1)) - Xe(3) + pi/2
                    % Calculate the residuals of the range and bearing
                    z  = [[sqrt(conOOI(c, 1)^2 + conOOI(c, 2)^2) - ExpectedRange]; 
                          [wrapToPi(atan2(conOOI(c, 2), conOOI(c, 1)) - Xe(3) + pi/2 - ExpectedBearing)]];
                    
                    % Calculate the uncertainty covariance in measurements
                    R = diag([stdDevRange*stdDevRange*4 stdDevBearing*stdDevBearing*4]);
                    
                    % EKF update with measured ranges & bearings
                    [Xe, P] = DoUpdate(P, Xe, H, R, z);
                    
                    T0 = dt;
                end
            end
            % Show the range of LiDAR scan in polar coordinates
            set(hL,'ydata',r);
            
            set(ehand4, 'xdata', Xe(1), 'ydata', Xe(2));
            % Show the brilliant points in red (Intensity > 0)
            ii=find(I>0); set(hL2,'xdata',angleScan(ii),'ydata',r(ii));
            
            % Introduce a short pause in order to visualise scans
            % pause(0.01); 
        end
        XeH(:, i) = Xe;
    end
    
    % Plot the overall path of the UGV estimated by the kinematic model
    figure(5); hold on; grid on;
    title('UGV Path');
    xlabel('x(m)');
    ylabel('y(m)');
    axis([-1.5 3.5 -1 5]);
    plot(X(1, :), X(2, :), 'r');
    plot(XeH(1, :), XeH(2, :), 'g');
    
    % Plot the overall heading of the UGV estimated by the kinematic model
    figure(6); hold on; grid on;
    title('Heading');
    axis([0 250 -350 100]);
    plot(0.0001*double(Data.t), rad2deg(X(3, 1:end-1)), 'b');
    plot(0.0001*double(Data.t), rad2deg(XeH(3, 1:end-1)), 'g');
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
function connOOI = ProcessLiDAR( r, I , X )
    % Define global handles for plots
    global lhand lhand2 lhand3 ghand ghand2 ghand3 ghand4 ghand5 ghand6 ghand7 ghand8 ghand9 ghand10 ghand11 ghand12 ghand13 ghand14 Landmarks;
    
    % Convert the polar coordinates of the LiDAR into cartesian
    % coordinates using trigonometry and set these on the plot
    ang = 0:0.5:180;
    x = transpose(r) .* cos(deg2rad(ang));
    y = transpose(r) .* sin(deg2rad(ang));
    set(lhand,'xdata', x, 'ydata', y);
    ii=find(I>0);
    set(lhand2,'xdata',x(ii),'ydata',y(ii));
    
    % Convert the local coordinates into global using the LiDAR
    % displacement, a rotation transformation and linear transformation
    xc = x;
    yc = y + 0.46;
    xg = xc * cos(X(3) - pi/2) - yc * sin(X(3) - pi/2) + X(1);
    yg = xc * sin(X(3) - pi/2) + yc * cos(X(3) - pi/2) + X(2);
    set(ghand,'xdata', xg, 'ydata', yg);
    set(ghand2,'xdata', xg(ii),'ydata', yg(ii));
    
    % Plot the UGVs estimated current postion using the kinematic model
    set(ghand4, 'xdata', X(1), 'ydata', X(2));

    % Used to calculate the postion of the OOIs
    ooi = zeros(1, 2);
    n = 1; m = 1;
    
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
    
    % Initialise data association matrix
    connOOI = zeros(Landmarks.n, 2);
    
    % Check that there was an OOI recorded in the scan
    if (ooi(1,1) ~= 0 && ooi (1,2) ~= 0)
        % Transform the OOIs into the global frame of reference
        ooiyc(:,2) = ooi(:,2) + 0.46;
        ooixg = ooi(:,1) * cos(X(3) - pi/2) - ooiyc(:,2) * sin(X(3) - pi/2) + X(1);
        ooiyg = ooi(:,1) * sin(X(3) - pi/2) + ooiyc(:,2) * cos(X(3) - pi/2) + X(2);

        % Set the global and local OOIs
        set(lhand3, 'xdata', ooi(:,1), 'ydata', ooi(:,2));
        set(ghand3, 'xdata', ooixg, 'ydata', ooiyg);
    
    % For all the landmarks provided
    for z = 1:Landmarks.n
        % For all OOIs detected
        for b = 1:length(ooixg)
            % Check the distance between them is 0.6m
            if(sqrt(((Landmarks.x(z) - ooixg(b))^2 + ((Landmarks.y(z) - ooiyg(b))^2))) < 0.6)
                % If it is store the Cartesian coordinates in the
                % associated landmarks matrix
                connOOI(z, 1) = ooixg(b);
                connOOI(z, 2) = ooiyg(b);
                break;
            end
        end
    end
    % Define OOI association handles
    handles = [ghand10, ghand11, ghand12, ghand13, ghand14];
    % Also set different colours for associated landmarks for better
    % visualisation
    for f=1:5
        if(connOOI(f,1) ~= 0 && connOOI(f,2) ~= 0)
            set(handles(f), 'xdata', connOOI(f,1), 'ydata', connOOI(f,2))
        else
            set(handles(f), 'xdata', -999999, 'ydata', -999999);
        end
    end
    % Function to set the landmark and connected OOI
    dataAssociation(ghand5, Landmarks.x(1), Landmarks.y(1), connOOI(1, :));
    dataAssociation(ghand6, Landmarks.x(2), Landmarks.y(2), connOOI(2, :));
    dataAssociation(ghand7, Landmarks.x(3), Landmarks.y(3), connOOI(3, :));
    dataAssociation(ghand8, Landmarks.x(4), Landmarks.y(4), connOOI(4, :));
    dataAssociation(ghand9, Landmarks.x(5), Landmarks.y(5), connOOI(5, :));
    
    end

end

% -------------------------------------------------------

% Function for processing and plotting the local and global cartesian coordinates of the UGV
function dataAssociation(handle, landmarkx, landmarky, OOI)
    % Check if the Landmark does have an associated OOI
    if (OOI(1) == 0 && OOI(2) == 0 )
        set(handle, 'xdata', 10000, 'ydata', 10000);
    else
        set(handle, 'xdata', [landmarkx, OOI(1)], 'ydata', [landmarky, OOI(2)]);
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