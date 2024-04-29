classdef lqr_control < handle
    properties(Access = public)
        k(4,12) double;
        k_faster(4,12) double;
        y_history double;
        u0(4,1) double;
        targets;
        uav_dist_list double
        y_hist_size(1,1) double;
        count(1,1) int64;
        sizecheck(1,1) double
        mindist (1,1) double
    end

    methods(Access = public)
        function obj = lqr_control(quadrotor, A, B, Q, R)
            obj.u0 = quadrotor.m*quadrotor.g/4;
            obj.k = lqr(A, B, Q, R);
            Q_faster=diag([1000,1000, 300, 5000, 5000, 5000, 1000, 1000, 1000, 50000, 50000, 50000]);
            obj.k_faster=lqr(A,B,Q_faster,R);
            obj.targets = [];
            obj.count = 0;
            obj.y_hist_size = 15;
            obj.y_history = zeros(3, obj.y_hist_size);
            obj.mindist = 1000;
            obj.uav_dist_list = zeros(1, 100);
        end

        function y_pred = curve_fit_2(obj, pointset, delta_t)
            datalen = size(pointset, 2);
            output_x = pointset(1, :);
            output_y = pointset(2, :);
            output_z = pointset(3, :);
            input_t = 0:datalen-1;

            coeff_x = polyfit(input_t, output_x, 2);
            coeff_y = polyfit(input_t, output_y, 2);
            coeff_z = polyfit(input_t, output_z, 2);

            pred_x = polyval(coeff_x, input_t(end) + delta_t);
            pred_y = polyval(coeff_y, input_t(end) + delta_t);
            pred_z = polyval(coeff_z, input_t(end) + delta_t);

            y_pred = [pred_x, pred_y, pred_z]';
        end

        function y=target_binder(obj,y_unbounded)
            box_size=10;
            drone_size=1.5;
            y=y_unbounded;
            x_max=box_size/2 -drone_size;
            y_max=box_size/2 -drone_size;
            z_max=box_size-drone_size;
            z_min=0.2;
            x_min=-box_size/2 +drone_size;
            y_min=-box_size/2 +drone_size;
            if y_unbounded(1)>x_max
                y(1)=x_max;
            end
            if y_unbounded(1)<x_min
               y(1)=x_min;
            end
            if y_unbounded(2)>y_max
               y(2)=y_max;
            end
            if y_unbounded(2)<y_min
               y(2)=y_min;
            end
            if y_unbounded(3)>z_max
               y(3)=z_max;
            end
            if y_unbounded(3)<z_min
               y(3)=z_min;
            end
        end

        function e=error_binder(obj,e_unbounded)
            max_norm=5;  
            e=e_unbounded;
            curr_norm=norm(e_unbounded(1:3));
            if curr_norm>max_norm
                e(1:3)=(max_norm/curr_norm)*e_unbounded(1:3);
            end
        end

        function u = output(obj, isCaptured, z, y0)
            uav_dist = norm(y0 - z(1:3))
            obj.sizecheck=obj.sizecheck+1;
            obj.y_history(:,1)=[];
            obj.y_history=[obj.y_history,y0];
            uav_vel=(obj.y_history(:,end)-obj.y_history(:,(end-1)))/0.01;%Time step is 0.01
            uav_speed=norm(uav_vel)
            target_dir=y0-z(1:3);
            if dot(uav_vel,target_dir)<0
                delta_t=20;
            else
                delta_t=10;
            end
            target_vel=[0,0,0]';
            k_final=obj.k;
            if isCaptured == false
                if obj.sizecheck < obj.y_hist_size
                    yt = y0;
                else
                    yt = obj.curve_fit_2(obj.y_history, delta_t);
                    if obj.sizecheck>900
                        %disp("vel_target_active")
                        target_vel=yt-z(1:3);
                        target_vel(3)=0;
                        target_vel=target_vel*(0.75*uav_speed+(obj.sizecheck/10000))/norm(target_vel);
                        k_final=obj.k_faster;
                    end
                   

                end
            else
                yt = [0, 0, 0]';
                target_vel=[0, 0, 0]';
                k_final=obj.k;% Return to origin if captured
            end
            yt = obj.target_binder(yt);
            obj.targets = [obj.targets; yt];
            error = obj.error_binder(z - [yt; zeros(3, 1);target_vel;zeros(3,1)]);
            u = obj.u0 + (-k_final * error);
        end
    end
end
