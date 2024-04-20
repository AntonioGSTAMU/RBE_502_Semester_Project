classdef lqr_control < handle
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
            
            %pred_w=0.8;
            if isCaptured==false
                if obj.count<obj.y_hist_size 
                    yt=y0;
                    obj.count=obj.count+1;
                else
                    %yt=y0+obj.second_order_pred(obj.y_history(:,1),obj.y_history(:,2),obj.y_history(:,3));%y0+110*(obj.y_history(:,2)-obj.y_history(:,3));
                    %yt=obj.curve_fit_2(obj.y_history, delta_t);
                    yt=obj.curve_fit_2(obj.y_history, delta_t);
                end
            else
                yt=[0 0 0]';
            end
            
            u = obj.u0 + (-obj.k*(z-[yt;zeros(9,1)]));
        end
    end


end