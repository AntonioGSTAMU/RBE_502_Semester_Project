classdef lqr_control < handle
    properties(Access = public)
        %target(1,3) double;
        k(4,12) double;
        k2(4,12) double;
        k3(4,12) double;
        k4(4,12) double;
        k5(4,12) double;
        k6(4,12) double;
        u0(4,1) double;
        y_history double;
        targets;
        y_hist_size(1,1) double;
        count(1,1) int64;
        sizecheck(1,1) int16
        mindist (1,1) double
    end


    methods(Access = public)
        function obj = lqr_control(quadrotor, A, B, Q, R) %target_coords
            obj.u0 = quadrotor.m*quadrotor.g/4;
            %obj.target = target_coords;
            %obj.target = obj.target';
            
            obj.k = lqr(A,B,Q,R);
            obj.targets=[];

            mult_fact2=20;
            mult_fact3=30;
            mult_fact4=50;
            mult_fact5=0.2;
            mult_fact6=2;

            Q2=Q;
            Q2(1,1)=mult_fact2*Q2(1,1);
            Q2(2,2)=mult_fact2*Q2(2,2);
            Q2(3,3)=mult_fact2*Q2(3,3);
            obj.k2= lqr(A,B,Q2,R);
            Q3=Q;
            Q3(1,1)=mult_fact3*Q3(1,1);
            Q3(2,2)=mult_fact3*Q3(2,2);
            Q3(3,3)=mult_fact3*Q3(3,3);
            obj.k3= lqr(A,B,Q3,R);            
            Q4=Q;
            Q4(1,1)=mult_fact4*Q4(1,1);
            Q4(2,2)=mult_fact4*Q4(2,2);
            Q4(3,3)=mult_fact4*Q4(3,3);
            obj.k4= lqr(A,B,Q4,R);            
            Q5=Q;
            Q5(1,1)=mult_fact5*Q5(1,1);
            Q5(2,2)=mult_fact5*Q5(2,2);
            Q5(3,3)=mult_fact5*Q5(3,3);
            obj.k5= lqr(A,B,Q5,R);
            Q6=Q;
            Q6(1,1)=mult_fact6*Q6(1,1);
            Q6(2,2)=mult_fact6*Q6(2,2);
            Q6(3,3)=mult_fact6*Q6(3,3);
            obj.k6= lqr(A,B,Q6,R);

            obj.count=0;
            obj.y_hist_size=15;
            obj.y_history=zeros(3, obj.y_hist_size);
            obj.mindist=1000;
        end

        function y_pred= curve_fit_2(obj, pointset, delta_t)
            %disp(size(pointset));
            datalen=size(pointset,2);
    

            output_x=pointset(1,:);
            output_y=pointset(2,:);
            output_z=pointset(3,:);
            input_t=0:datalen-1;

            coeff_x=polyfit(input_t,output_x,2);
            coeff_y=polyfit(input_t,output_y,2);
            coeff_z=polyfit(input_t,output_z,2);

            pred_x=polyval(coeff_x,input_t(end)+delta_t);
            pred_y=polyval(coeff_y,input_t(end)+delta_t);
            pred_z=polyval(coeff_z,input_t(end)+delta_t);

            y_pred=[pred_x,pred_y,pred_z]';

        end

        function y=target_binder(obj,y_unbounded)
            box_size=10;
            y=y_unbounded;
            x_max=box_size/2 -0.2;
            y_max=box_size/2 -0.2;
            z_max=box_size-0.2-0.2;
            z_min=0+0.2;
            x_min=-box_size/2 +0.2;
            y_min=-box_size/2 +0.2;
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


        function u = output(obj, isCaptured, z, y0) %z should be a 12x1 column vector
            if mod(obj.count,1)==0
                obj.sizecheck=obj.sizecheck+1;
                obj.y_history(:,1)=[];
                obj.y_history=[obj.y_history,y0];
            end
            delta_t=110;
            k_final=obj.k;
            %pred_w=0.8;
            %tic
            if isCaptured==false %Try to catch the uav until it is captured
                if obj.sizecheck<obj.y_hist_size %Until we have enough readings to predict, just follow the current position
                    yt=y0;
                
                else
                    uav_dist=norm(y0-z(1:3));
                    if uav_dist>1 && uav_dist<=3.5 %If we're close be more aggressive
                        k_final=obj.k;
                        yt=obj.curve_fit_2(obj.y_history, 1.1*delta_t);
                    elseif uav_dist<=0.4 && uav_dist>0.2
                        k_final=obj.k2;
                        yt=obj.curve_fit_2(obj.y_history, delta_t/2);
                    elseif uav_dist<=0.2 && uav_dist>0.1
                        k_final=obj.k3;
                        yt=obj.curve_fit_2(obj.y_history, delta_t/2);
                    elseif uav_dist<=0.1
                        k_final=obj.k4;
                        yt=obj.curve_fit_2(obj.y_history, delta_t/2);
                    elseif uav_dist>=3.5
                        k_final=obj.k5;
                        yt=obj.curve_fit_2(obj.y_history, delta_t);
                    elseif uav_dist<=1 && uav_dist>0.4
                        k_final=obj.k6;
                        yt=obj.curve_fit_2(obj.y_history, delta_t);
                    end
                end
            else %If captured, return to origin
                yt=[0 0 0]';
                k_final=obj.k5;

            end
            obj.count=obj.count+1;
            %disp(toc)
            yt=obj.target_binder(yt);
            obj.targets=[obj.targets;yt'];
            error=obj.error_binder(z-[yt;zeros(9,1)]);
            %error=z-[yt;zeros(9,1)];
            %disp(norm(error));
            u = obj.u0 + (-k_final*(error));

        end
    end


end