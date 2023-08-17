% This script runs Q3(a)

% Create the configuration object.
configuration = drivebot.SimulatorConfiguration();

% SLAM is enabled, GPS disabled
configuration.enableGPS = false;
configuration.enableLaser = true;

% If you set this parameter to false, the simulator generates measurements
% with no noise in them. You might find this useful for debugging.
% However, unless specified otherwise, any submitted results must have this
% value set to true.
configuration.perturbWithNoise = true;

% Magic tuning for the no-prediction case
configuration.laserDetectionRange = 30;

% Set up the simulator
simulator = drivebot.DriveBotSimulator(configuration, 'q3_a');

% Create the localization system
drivebotSLAMSystem = drivebot.DriveBotSLAMSystem(configuration);

% This tells the SLAM system to do a very detailed check that the input
% appears to be correct but can make the code run slowly. Once you are
% confident your code is working safely, you can set this to false.
drivebotSLAMSystem.setValidateGraph(false);

% Optimize every 500 timesteps to get a picture of the situation evolves
drivebotSLAMSystem.setRecommendOptimizationPeriod(500);

% Set whether the SLAM system should remove prediction edges. If the first
% value is true, the SLAM system should remove the edges. If the second is
% true, the first prediction edge will be retained.
%drivebotSLAMSystem.setRemovePredictionEdges(true, false);

%% 3b
drivebotSLAMSystem.setPruning(true);
%% 3b

% Run the main loop and correct results
results = minislam.mainLoop(simulator, drivebotSLAMSystem);

% Minimal output plots. For your answers, please provide titles and label
% the axes.

% Plot optimisation times
minislam.graphics.FigureManager.getFigure('Optimization times');
clf
plot(results{1}.optimizationTimes, '*')
title('Optimization times')
xlabel('Time Step'); ylabel('Time taken (seconds)');
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

% Plot the chi2 value down here. The chi2 values are logged internally and
% can be plotted directly down here.
% chi2 is the sum of the terms e^T*Omega*e which is a measure of
% consistency of performance of the SLAM algorithm and shows 
% how far away the vehicle is from the mean of the distribution.

minislam.graphics.FigureManager.getFigure('chi2');
clf;
plot(results{1}.chi2Time',results{1}.chi2History');
title('Plot of chi-squared values showing performance consistency');
xlabel('Time (s)'); ylabel('chi2 values');
hold on

