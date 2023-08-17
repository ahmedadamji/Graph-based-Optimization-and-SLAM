% This script runs Q1(c)

% Create the configuration object.
configuration = drivebot.SimulatorConfiguration();

% Since we are doing prediction and GPS, disable the SLAM sensor
configuration.enableGPS = true;
configuration.enableLaser = false;

% If you set this parameter to false, the simulator generates measurements
% with no noise in them. You might find this useful for debugging.
% However, unless specified otherwise, any submitted results must have this
% value set to true.
configuration.perturbWithNoise = true;

% Set up the simulator
simulator = drivebot.DriveBotSimulator(configuration, 'q1_bc');

% Create the localization system
drivebotSLAMSystem = drivebot.DriveBotSLAMSystem(configuration);

% This tells the SLAM system to do a very detailed check that the input
% appears to be correct but can make the code run slowly. Once you are
% confident your code is working safely, you can set this to false.
drivebotSLAMSystem.setValidateGraph(true);

% Run the main loop and correct results
results = minislam.mainLoop(simulator, drivebotSLAMSystem);

% Minimal output plots. For your answers, please provide titles and label
% the axes.

% Plot optimisation times
minislam.graphics.FigureManager.getFigure('Optimization times');
clf
plot(results{1}.optimizationTimes, '*')
title('Optimization times')
xlabel('Optimsation index'); ylabel('Time taken (seconds)');
hold on

% % Plot the error curves
% minislam.graphics.FigureManager.getFigure('Errors');
% clf
% plot(results{1}.vehicleStateHistory'-results{1}.vehicleStateHistory')

% Plot covariance
% covariance is a 3 by N dimensional vector where the nth column 
% are the diagonals from the covariance matrix.
minislam.graphics.FigureManager.getFigure('Vehicle Covariances');
clf;
yyaxis left
plot(results{1}.vehicleCovarianceHistory(1:2,:)')
yyaxis right
plot(results{1}.vehicleCovarianceHistory(3, :)')
title('Vehicle Covariances');
legend('Px','Py','Ptheta');
xlabel('Time Step'); ylabel('Vehicle Covariances');
hold on

% Plot errors
% X is a 3 by N dimensional vector of vehicle state (x, y, theta)
minislam.graphics.FigureManager.getFigure('Errors');
clf;
yyaxis left
plot(results{1}.vehicleStateHistory(1:2,:)'-results{1}.vehicleTrueStateHistory(1:2,:)');
yyaxis right
plot(results{1}.vehicleStateHistory(3, :)'-results{1}.vehicleTrueStateHistory(3, :)');
title('Errors');
legend('x','y','theta');
xlabel('Time Step'); ylabel('Vehicle State Estimate Error');
hold on




