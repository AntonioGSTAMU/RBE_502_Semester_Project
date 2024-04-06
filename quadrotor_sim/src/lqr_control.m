classdef lqr_control < handle
    properties(Access = public)
        target(3,1) double;
        k(1,2) double;
        u0(1,1) double;
    end


    methods(Access = public)
        function obj = lqr_control(target_coords, quadrotor, A, B, Q, R)
            obj.u0 = quadrotor.m*quadrotor.g/4;
            obj.target = target_coords;
            obj.k = lqr(A,B,Q,R);
        end

        function u = output(obj, ~, z, ~) %z should be a 12x1 column vector
            u = obj.u0 + (-obj.k*[(z(1:3)-obj.target);z(4:12)]); %repmat(obj.k*[(obj.altitude - z(3)); -z(9)],[4,1]);
        end
    end


end