classdef lqr_control_v1 < handle
    properties(Access = public)
        %target(1,3) double;
        k(4,12) double;
        u0(4,1) double;
        y_history double;
        y_hist_size(1,1) double;
        count(1,1) double;
        mindist (1,1) double
    end


    methods(Access = public)
        function obj = lqr_control(quadrotor, A, B, Q, R) %target_coords
            obj.u0 = quadrotor.m*quadrotor.g/4;
            %obj.target = target_coords;
            %obj.target = obj.target';
            obj.k = lqr(A,B,Q,R);
            obj.count=0;
            obj.y_hist_size=5;
            obj.y_history=zeros(3,3);
            obj.mindist=1000;
        end

        function del_y= first_order_pred(obj, y_0,y_1)
            del_y=110*(y_1-y_0);
        end
        function del_y= second_order_pred(obj, y_0,y_1, y_2)
            del_y_2=y_2-y_1;
            del_y_1=y_1-y_0;
            del_y_dash=del_y_2-del_y_1;
            del_y=100*(del_y_2+del_y_dash);
        end

        function u = output(obj, isCaptured, z, y0) %z should be a 12x1 column vector
            obj.y_history(:,1)=[];
            obj.y_history=[obj.y_history,y0];
            if isCaptured==false
                if obj.count<obj.y_hist_size 
                    yt=y0;
                    obj.count=obj.count+1;
                else
                    yt=y0+obj.second_order_pred(obj.y_history(:,1),obj.y_history(:,2),obj.y_history(:,3));%y0+110*(obj.y_history(:,2)-obj.y_history(:,3));
                end
            else
                yt=[0 0 0]';
            end
            
            %disp(yt-y0);
            %obj.y_history=[];
            %yt=
            %const_vel_factor=-0.1*normalize(y0-z(1:3));
            u = obj.u0 + (-obj.k*(z-[yt;zeros(9,1)])); %u = obj.u0 + (-obj.k*[(z(1:3)-obj.target);z(4:12)]); %repmat(obj.k*[(obj.altitude - z(3)); -z(9)],[4,1]);
            %u = obj.u0 + (-obj.k*(z-[yt;zeros(3,1);const_vel_factor;zeros(3,1)]));%
        end
    end


end