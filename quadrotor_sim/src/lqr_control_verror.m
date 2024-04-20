classdef lqr_control < handle
    properties(Access = public)
        %target(1,3) double;
        k(4,12) double;
        k2(4,12) double;
        k3(4,12) double;
        k4(4,12) double;
        k5(4,12) double;
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
            mult_fact2=10;
            mult_fact3=50;
            mult_fact4=100;
            mult_fact5=200;
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

            obj.count=0;
            obj.y_hist_size=15;
            obj.y_history=zeros(3, obj.y_hist_size);
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

        function u = output(obj, isCaptured, z, y0) %z should be a 12x1 column vector
            obj.y_history(:,1)=[];
            obj.y_history=[obj.y_history,y0];
            delta_t=110;
            k_final=obj.k;
            %pred_w=0.8;
            if isCaptured==false %Try to catch the uav until it is captured
                if obj.count<obj.y_hist_size %Until we have enough readings to predict, just follow the current position
                    yt=y0;
                    obj.count=obj.count+1;
                else
                    if norm(y0-z(1:3))<0.2 %If we're close be more aggressive
                        k_final=obj.k;
                        yt=obj.curve_fit_2(obj.y_history, delta_t);
                        
                    else
                        yt=obj.curve_fit_2(obj.y_history, delta_t);
                    end
                end
            else %If captured, return to origin
                yt=[0 0 0]';
            end

            if norm(y0-z(1:3))>0.3
                u = obj.u0 + (-k_final*(z-[yt;zeros(9,1)]));
            elseif isCaptured==false
                v_error=1*(y0-z(1:3))/norm(y0-z(1:3));
                u = obj.u0 + (-k_final*(z-[yt;zeros(3,1);v_error;zeros(3,1)]));
            else
                u = obj.u0 + (-k_final*(z-[yt;zeros(9,1)]));
            end
        end
    end


end