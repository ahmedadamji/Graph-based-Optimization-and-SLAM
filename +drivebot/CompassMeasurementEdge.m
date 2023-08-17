classdef CompassMeasurementEdge < g2o.core.BaseUnaryEdge
   
    methods(Access = public)
    
        function this = CompassMeasurementEdge()
            this = this@g2o.core.BaseUnaryEdge(1);
        end
        
        function computeError(this)
            %Compute the error used to compute the edge
            x = this.edgeVertices{1}.estimate();
            this.errorZ = g2o.stuff.normalize_theta(x(3) - this.z);
        end
        
        function linearizeOplus(this)
            %Compute the jacobian of the edge
            this.J{1} = [0 0 1];
        end        
    end
end