classdef GPSMeasurementEdge < g2o.core.BaseUnaryEdge
    
    methods(Access = public)
        
        function this = GPSMeasurementEdge()
            this = this@g2o.core.BaseUnaryEdge(2);
        end
        
        function computeError(this)
%             warning('gpsmeasurementedge:computeerror:unimplemented', ...
%                 'Implement the rest of this method for Q1c.');
            %Compute the error used to compute the edge
            %Edited to complete Q1c
            x = this.edgeVertices{1}.estimate();
            this.errorZ = x(1:2) - this.z;
        end
        
        function linearizeOplus(this)
%             warning('gpsmeasurementedge:linearizeplus:unimplemented', ...
%                 'Implement the rest of this method for Q1c.');
%            Compute the jacobian of the edge
            %Edited to complete Q1c
            this.J{1} = ...
                [1 0 0;
                0 1 0];
        end
    end
end