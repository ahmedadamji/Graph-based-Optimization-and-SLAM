% This script runs Q2(d)

% Create the configuration object.
configuration = drivebot.SimulatorConfiguration();

% Disable the GPS and enable the laser for pure SLAM.
configuration.enableGPS = false;
configuration.enableLaser = true;

% For this part of the coursework, this should be set to zero.
configuration.perturbWithNoise = false;

% Set this value to truncate the run at a specified timestep rather than
% run through the whole simulation to its end.
configuration.maximumStepNumber = 1208;  % 1207 is after loop closure,
                                         % 1206 before loop closure

% Set up the simulator
simulator = drivebot.DriveBotSimulator(configuration, 'q2_d');

% Create the localization system
drivebotSLAMSystem = drivebot.DriveBotSLAMSystem(configuration);
drivebotSLAMSystem.setRecommendOptimizationPeriod(inf);

% This tells the SLAM system to do a very detailed check that the input
% appears to be correct but can make the code run slowly. Once you are
% confident your code is working safely, you can set this to false.
drivebotSLAMSystem.setValidateGraph(false);

% Run the main loop and correct results
results = minislam.mainLoop(simulator, drivebotSLAMSystem);

% Minimal output plots. For your answers, please provide titles and label
% the axes.

% % Plot optimisation times
% minislam.graphics.FigureManager.getFigure('Optimization times');
% clf
% plot(results{1}.optimizationTimes, '*')
% title('Optimization times')
% xlabel('Optimsation index'); ylabel('Time taken (seconds)');
% hold on
% 
% % Plot covariance
% % covariance is a 3 by N dimensional vector where the nth column 
% % are the diagonals from the covariance matrix.
% minislam.graphics.FigureManager.getFigure('Vehicle Covariances');
% clf;
% yyaxis left
% plot(results{1}.vehicleCovarianceHistory(1:2,:)')
% yyaxis right
% plot(results{1}.vehicleCovarianceHistory(3, :)')
% title('Vehicle Covariances');
% legend('Px','Py','Ptheta');
% xlabel('Optimsation Index'); ylabel('Vehicle Covariances');
% hold on
% 
% % Plot errors
% % X is a 3 by N dimensional vector of vehicle state (x, y, theta)
% minislam.graphics.FigureManager.getFigure('Errors');
% clf;
% yyaxis left
% plot(results{1}.vehicleStateHistory(1:2,:)'-results{1}.vehicleTrueStateHistory(1:2,:)');
% yyaxis right
% plot(results{1}.vehicleStateHistory(3, :)'-results{1}.vehicleTrueStateHistory(3, :)');
% title('Errors');
% legend('x','y','theta');
% xlabel('Optimsation Index'); ylabel('Vehicle State Estimate Error');
% hold on

%Calculating difference in vehicle covariances before loop closure event:
VehicleCovarianceBeforeLoopClosure = results{1}.vehicleCovarianceHistory(:,1206)';
%Finding the matrix of vehicle covariances before loop closure event:
matrixOfVehicleCovarianceBeforeLoopClosure = eye(3,3).*VehicleCovarianceBeforeLoopClosure;
%Finding the determinant of the matrix of vehicle covariances before loop closure event:
determinantOfCovarianceBeforeLoopClosure = det(matrixOfVehicleCovarianceBeforeLoopClosure);
%displaying the results
disp(determinantOfCovarianceBeforeLoopClosure);

%Calculating difference in vehicle covariances after loop closure event:
VehicleCovarianceAfterLoopClosure = results{1}.vehicleCovarianceHistory(:,1207)';
%Finding the matrix of vehicle covariances after loop closure event:
matrixOfVehicleCovarianceAfterLoopClosure = eye(3,3).*VehicleCovarianceAfterLoopClosure;
%Finding the determinant of the matrix of vehicle covariances after loop closure event:
determinantOfCovarianceAfterLoopClosure = det(matrixOfVehicleCovarianceAfterLoopClosure);
%displaying the results
disp(determinantOfCovarianceAfterLoopClosure);


disp(determinantOfCovarianceBeforeLoopClosure-determinantOfCovarianceAfterLoopClosure);
