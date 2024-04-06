classdef lqr_control < handle
    properties(Access = public)
        %target(1,3) double;
        k(4,12) double;
        u0(4,1) double;
        y_history(3,3) double
    end


    methods(Access = public)
        function obj = lqr_control(quadrotor, A, B, Q, R) %target_coords
            obj.u0 = quadrotor.m*quadrotor.g/4;
            %obj.target = target_coords;
            %obj.target = obj.target';
            obj.k = lqr(A,B,Q,R);
        end

        function u = output(obj, ~, z, y0)%z should be a 12x1 column vector
            %obj.y_history=[];
            %yt=
            u = obj.u0 + (-obj.k*(z-[y0;zeros(9,1)])); %u = obj.u0 + (-obj.k*[(z(1:3)-obj.target);z(4:12)]); %repmat(obj.k*[(obj.altitude - z(3)); -z(9)],[4,1]);
        end
    end


end