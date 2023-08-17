% This class implements an event-based estimation system using g2o and
% the barebones for building up a minimal, ideal SLAM system. The system is
% event-based and responds to a sequence of events which are time stamped
% and served in order. To implement your SLAM system, you will need to
% implement various methods which mostly involve working with the graph.
% These methods are initially stubbed out and will generate exceptions if
% you try to call them.

classdef DriveBotSLAMSystem < minislam.slam.SLAMSystem

    properties(Access = public, Constant)
        % Platform state dimension
        NP = 3;

        % Landmark dimension
        NL = 2;

        % Initial cache size; might help a bit with performance
        INITIAL_CACHE_SIZE = 10000;
    end

    properties(Access = protected)

        % The most recently created vehicle vertex.
        currentVehicleVertex;

        % The set of all vertices associated with the vehicle state over
        % time.
        vehicleVertices;
        vehicleVertexId;

        % The set of all prediction edges. These are removed from the graph
        % afterwards if we don't use prediction
        processModelEdges;
        numProcessModelEdges;

        % The landmark vertices. Confusingly enough, "Map" here refers to
        % the data structure which is used to store the landmarks. (It
        % allows random access of landmarkID to landmark object.)
        landmarkIDStateVectorMap;

        % How often we recommend running the optimization
        recommendOptimizationPeriod;

        % Flag to show if we should prune the edges. This is needed for
        % question Q3a
        removePredictionEdgesFromGraph;
        keepFirstPredictionEdge;
        % Flag to show if we should prune the vertices. This is needed for
        % question Q3b
        pruneGraph;

        % Variables to show the total number of edges and vertices deleted
        % from the graph in question Q3b.
        totalVerticesRemovedFromGraph;
        totalEdgesRemovedFromGraph;

    end

    methods(Access = public)

        % Create the localization system and start it up.
        function this = DriveBotSLAMSystem(configuration)

            % Call the base class constructor
            this = this@minislam.slam.SLAMSystem(configuration);

            % Preallocate for convenience
            this.vehicleVertices = cell(1, this.INITIAL_CACHE_SIZE);

            % No vehicle vertices initally set
            this.vehicleVertexId = 0;

            % The set of prediction edges, initially empty
            this.processModelEdges = cell(1, this.INITIAL_CACHE_SIZE);
            this.numProcessModelEdges = 0;

            % Allocate the landmark map
            this.landmarkIDStateVectorMap = containers.Map('KeyType', 'int64', 'ValueType', 'any');

            % By default, run very infrequently
            this.recommendOptimizationPeriod = inf;

            this.removePredictionEdgesFromGraph = false;
            this.keepFirstPredictionEdge = false;

            %These changes were added to answer Q3b
            % initialising as false to not prune vertices or edges by default
            this.pruneGraph = false;
            % No vertices initally removed
            this.totalVerticesRemovedFromGraph = 0;
            % No edges initally removed
            this.totalEdgesRemovedFromGraph = 0;
        end

        % Recommend if an optimization is a good idea. Based on an event,
        % some activities (e.g., such as loop closing) can have a very big
        % impact on the estimates. The logic we have here just recommends
        % an optimization if a fixed number of steps have been completed.

        function recommendation = recommendOptimization(this)

            % This is how to do it after every 100 steps
            recommendation = rem(this.stepNumber, ...
                this.recommendOptimizationPeriod) == 0;
        end

        % Set the value of how often recommend optimization should return
        % true
        function setRecommendOptimizationPeriod(this, newRecommendOptimizationPeriod)
            this.recommendOptimizationPeriod = newRecommendOptimizationPeriod;
        end

        % Return the current mean and covariance estimate of the robot.
        % This is only valid after optimization has been called.
        function [x, P] = platformEstimate(this)
            [xS, PS] = this.graph.computeMarginals(this.currentVehicleVertex);
            x=full(xS);
            P=full(PS);
        end

        % Returns the entire history of the platform estimates. Suppose
        % there are n vehicle vertices. T is a 1 by N dimensional vector of
        % timesteps. X is a 3 by N dimensional vector of vehicle state (x,
        % y, theta). P is a 3 by N dimensional vector where the nth column
        % are the diagonals from the covariance matrix.
        function [T, X, P] = platformEstimateHistory(this)

            % Extract the graph
            [xS, PS] = this.graph.computeMarginals();

            % Create the output array
            X = zeros(this.NP, this.vehicleVertexId);
            P = zeros(this.NP, this.vehicleVertexId);
            T = zeros(1, this.vehicleVertexId);

            % Copy the outputs over
            for v = 1 : this.vehicleVertexId
                idx = this.vehicleVertices{v}.hessianIndex();

                T(v) = this.vehicleVertices{v}.time();

                % Copy the estimate into the array. If the vertices is
                % fixed (conditioned), its estimate is okay. The covariance
                % is not explicitly defined, but has a value of zero.
                % Therefore we fill this manually.
                if (isempty(idx) == true)
                    X(:, v) = this.vehicleVertices{v}.estimate();
                    P(:, v) = zeros(3, 1);
                else
                    X(:, v) = full(xS(idx));
                    P(:, v) = full(diag(PS(idx, idx)));
                end
            end
        end

        % Return the means and covariances of the landmark estimates. These
        % are only valid after optimization has been called.
        function [x, P, landmarkIds] = landmarkEstimates(this)

            landmarkVertices = values(this.landmarkIDStateVectorMap);

            numberOfLandmarks = length(landmarkVertices);

            landmarkIds = NaN(1, numberOfLandmarks);
            x = NaN(this.NL, numberOfLandmarks);
            P = NaN(this.NL, this.NL, numberOfLandmarks);

            [xS, PS] = this.graph.computeMarginals();

            for l = 1 : numberOfLandmarks
                landmarkIds(l) = landmarkVertices{l}.landmarkId();
                idx = landmarkVertices{l}.hessianIndex();
                x(:, l) = full(xS(idx));
                if (isempty(idx == true))
                    P(:, :, l) = zeros(3, 3);
                else
                    P(:, :, l) = full(PS(idx, idx));
                end
            end
        end

        % We overload the optimize method so that you can add additional
        % logic here
        function chi2 = optimize(this, maximumNumberOfOptimizationSteps)

            % Remove the prediction edges if requested.
            %These changes were added to answer Q3a
            if ((this.removePredictionEdgesFromGraph == true) || (this.keepFirstPredictionEdge == true))
                this.deleteVehiclePredictionEdges();
            end

            if (this.pruneGraph == true)
                this.deleteVehicleVertices();
            end


            % Now call the actual optimizer. Let it handle the default if
            % no steps are specified.
            if (nargin > 1)
                chi2 = optimize@minislam.slam.SLAMSystem(this, ...
                    maximumNumberOfOptimizationSteps);
            else
                chi2 = optimize@minislam.slam.SLAMSystem(this);
            end
        end

        function setRemovePredictionEdges(this, removeEdges, keepFirst)
            this.removePredictionEdgesFromGraph = removeEdges;
            this.keepFirstPredictionEdge = keepFirst;

        end

        function setPruning(this, prune)
            this.pruneGraph = prune;

        end

    end

    % These are the methods you will need to overload
    methods(Access = protected)

        % Handle the initial condition

        function handleInitialConditionEvent(this, event)

            % Create the first vertex, set its estimate to the initial
            % value and add it to the graph.
            this.currentVehicleVertex = drivebot.VehicleStateVertex(this.currentTime);
            this.currentVehicleVertex.setEstimate(event.data);
            this.graph.addVertex(this.currentVehicleVertex);

            % Set the book keeping for this initial vertex.
            this.vehicleVertexId = 1;
            this.vehicleVertices{this.vehicleVertexId} = this.currentVehicleVertex;

            % If the covariance is 0, the vertex is known perfectly and so
            % we set it as fixed. If the covariance is non-zero, add a
            % unary initial prior condition edge instead. This adds a soft
            % constraint on where the state can be.
            if (det(event.covariance) < 1e-6)
                this.currentVehicleVertex.setFixed(true);
            else
                initialPriorEdge = drivebot.InitialPriorEdge();
                initialPriorEdge.setMeasurement(event.data);
                initialPriorEdge.setInformation(inv(event.covariance));
                initialPriorEdge.setVertex(this.currentVehicleVertex);
                this.graph.addEdge(initialPriorEdge);
            end
        end

        function handleNoPrediction(~)
            % Nothing to do
        end

        function handleHeartbeatEvent(this, ~)
            % Nothing to do
        end

        function handlePredictToTime(this, time, dT)

            % Create the next vehicle vertex and add it to the graph

            this.currentVehicleVertex = drivebot.VehicleStateVertex(time);
            %this.currentVehicleVertex.estimate;

            %These changes were added to answer Q1b:
            
            % Implement prediction code here
            this.graph.addVertex(this.currentVehicleVertex);

            %Defining the process model
            % Work out the information matrices of the Odometry covariance
            omegaU = inv(this.uCov);
            
            %Getting the current vehicle state estimates
            X = this.vehicleVertices{this.vehicleVertexId}.estimate;
            
            %Computing the process noise which is a zero mean gaussian
            v_k = sqrtm(this.uCov) * randn(3, 1);
            
            %Computing the rotation matrix from the vehicle frame to the
            %world frame
            M_k = [cos(X(3)) -sin(X(3)) 0;...
                sin(X(3)) cos(X(3)) 0;...
                0 0 1];

            %Computing the update where dT is the lenght of the prediction
            %interval
            %this.u consists of the speed of the vehicle and the angular velocity
            %term
            update = (dT * M_k * (this.u + v_k));

            %Setting the current estimate to be the current vehicle vertex
            this.currentVehicleVertex.setEstimate(X);

            %Using the oplus method to perform prediction update
            this.currentVehicleVertex.oplus(update);

            % Create the edge
            e = drivebot.VehicleKinematicsEdge(dT);

            % Link it so that it connects to the vertex we want to estimate
            e.setVertex(1, this.vehicleVertices{this.vehicleVertexId});
            e.setVertex(2,this.currentVehicleVertex);

            % Set the measurement value and the measurement covariance
            e.setMeasurement(this.u);
            e.setInformation(omegaU);

            % Add the edge to the graph
            this.graph.addEdge(e);


            %warning('drivebotslam:handlepredicttotime:unimplemented', ...
            %    'Implement the rest of this method for Q1b.');


            % Bump the indices
            this.vehicleVertexId = this.vehicleVertexId + 1;
            this.vehicleVertices{this.vehicleVertexId} = this.currentVehicleVertex;
        end

        function handleGPSObservationEvent(this, event)

            % Create a GPS measurement edge

            % Q1c:
            % Implement prediction code here
            %             warning('drivebotslam:handlegpsobservationevent:unimplemented', ...
            %                 'Implement the rest of this method for Q1c.');
            %%
            
             %This is for  Q1c:
            % Create the edge
            e = drivebot.GPSMeasurementEdge();

            % Link it so that it connects to the vertex we want to estimate
            e.setVertex(1, this.vehicleVertices{this.vehicleVertexId});

            % Set the measurement value and the measurement covariance
            e.setMeasurement(event.data);
            e.setInformation(inv(event.covariance));

            % Add the edge to the graph
            this.graph.addEdge(e);


        end

        function handleLandmarkObservationEvent(this, event)

            % Iterate over all the landmark measurements
            for l = 1 : length(event.landmarkIds)

                % Get the landmark vertex associated with this measurement.
                % If necessary, a new landmark vertex is created and added
                % to the graph.
                [landmarkVertex, newVertexCreated] = this.createOrGetLandmark(event.landmarkIds(l));
                z = event.data(:, l);

                % Finish to implement Q2b

                % warning('drivebotslamsystem:handlelandmarkobservationevent:unimplemented', ...
                % 'Implement the rest of this method for Q2b.');
                
                %These changes were added to answer Q2b

                %Create measurement Edge
                landmarkRangeBearingEdge = drivebot.LandmarkRangeBearingEdge();

                % Link it so that it connects to the vertex we want to estimate
                landmarkRangeBearingEdge.setVertex(1,this.vehicleVertices{this.vehicleVertexId});
                landmarkRangeBearingEdge.setVertex(2,this.landmarkIDStateVectorMap(event.landmarkIds(l)));

                % Set the measurement value and the measurement covariance
                landmarkRangeBearingEdge.setMeasurement(z);
                landmarkRangeBearingEdge.setInformation(inv(event.covariance));


                if (newVertexCreated == true)
                    landmarkRangeBearingEdge.initialize();
                end
                
                this.graph.addEdge(landmarkRangeBearingEdge);
            end
        end

        function deleteVehiclePredictionEdges(this)
            % Q3a:
            %             warning('drivebotslam:deletevehiclepredictionedges:unimplemented', ...
            %                 'Implement the rest of this method for Q3a.');
            
            %These changes were added to answer Q3a
            graph_edges = this.graph.edges();

            totalEdges = length(graph_edges);
            num_vehicle_edges = 0;
            
            %Iterate through the the total number of edges and check if it
            %is a vehicle kinematics edge. If it is, delete all the prediction edges
            %except when required to keep only the first
            for i = 1:totalEdges
                if isa(graph_edges{i},"drivebot.VehicleKinematicsEdge")
                    %If this.keepFirstPredictionEdge is true, keep the
                    %first ede
                    if num_vehicle_edges == 0 && this.keepFirstPredictionEdge
                        disp('Keept First Prediction Edge');
                        num_vehicle_edges = num_vehicle_edges + 1;
                    else
                        this.graph.removeEdge(graph_edges{i});
                        num_vehicle_edges = num_vehicle_edges + 1;
                    end
                end
            end

        end



        function deleteVehicleVertices(this)
            %These changes were added to answer Q3b
            
            %iterate through all the vehicle vertices
            for i = 1:this.vehicleVertexId
                
                vertex = this.vehicleVertices{i};

                vertexEdges = vertex.edges();

                %Obtain total number of edges attached to the vertex
                totalEdgesOnVertex = length(vertexEdges);

                totalLandmarkEdgesOnVertex = 0;

                %iterate through a the edges on the vertex and calculate
                %how many of them are landmark range bearing edges.
                for j = 1:totalEdgesOnVertex
                    if isa(vertexEdges{j},"drivebot.LandmarkRangeBearingEdge")
                        totalLandmarkEdgesOnVertex = totalLandmarkEdgesOnVertex + 1;      
                    end
                end
                
                %If the landmark egdes are less than 3 delete all the edges
                %starting from the second edge
                if ((totalLandmarkEdgesOnVertex < 3) && (i > 1))
                    vertexEdges = vertex.edges();
                    totalEdgesOnVertex = length(vertexEdges);
                    for k = 1:totalEdgesOnVertex
                        if isa(vertexEdges{k},"drivebot.LandmarkRangeBearingEdge")
                            this.graph.removeEdge(vertexEdges{k});
                            this.totalEdgesRemovedFromGraph = this.totalEdgesRemovedFromGraph + 1;
                        end
                        
                    end

                end

            end

            
            totalVerticesOnGraph = length(this.graph.vertices());
            totalEdgesOnGraph = length(this.graph.edges());
            
            disp("Total number of vertices removed from the graph: ")
            disp(this.totalVerticesRemovedFromGraph);
            disp("Total number of edges removed from the graph: ")
            disp(this.totalEdgesRemovedFromGraph);
            disp("Total number of vertices in the graph: ");
            disp(totalVerticesOnGraph);
            disp("Total number of edges in the graph: ");
            disp(totalEdgesOnGraph);

            disp('finished Q3b');
        end

        % This method returns a landmark associated with landmarkId. If a
        % landmark exists already, it is returned. If it does not exist, a
        % vertex is created and is added to the graph.
        function [landmarkVertex, newVertexCreated] = createOrGetLandmark(this, landmarkId)

            % If the landmark exists already, return it
            if (isKey(this.landmarkIDStateVectorMap, landmarkId) == true)
                landmarkVertex = this.landmarkIDStateVectorMap(landmarkId);
                newVertexCreated = false;
                return
            end

            fprintf('Creating landmark %d\n', landmarkId);

            % Create the new landmark add it to the graph
            landmarkVertex = drivebot.LandmarkStateVertex(landmarkId);
            this.landmarkIDStateVectorMap(landmarkId) = landmarkVertex;

            this.graph.addVertex(landmarkVertex);

            newVertexCreated = true;
        end

        function storeStepResults(this)
            % Nothing
        end

    end
end
