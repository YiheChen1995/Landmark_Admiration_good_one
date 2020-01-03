


function [CT_THETAS,CT_OUTER_TANGENT_POINTS] = dema_tangent_points(CT_ROUTE,detection_range)
         CT_THETAS = [];
         CT_OUTER_TANGENT_POINTS = [];

         lc_set_num = length(CT_ROUTE)-1;
         for i=1:lc_set_num
             lc_p1 = CT_ROUTE(i,:);
             lc_p2 = CT_ROUTE(i+1,:);
             lc_theta = atan((lc_p2(2)-lc_p1(2))/(lc_p2(1)-lc_p1(1)));
             %disp(lc_p2(2))
             CT_THETAS = [CT_THETAS;lc_theta];
             lc_array_x = [
                 lc_p1(1)+detection_range*sin(lc_theta);
                 lc_p1(1)-detection_range*sin(lc_theta);
                 lc_p2(1)+detection_range*sin(lc_theta);
                 lc_p2(1)-detection_range*sin(lc_theta);
              ];
             lc_array_y = [
                 lc_p1(2)-detection_range*cos(lc_theta);
                 lc_p1(2)+detection_range*cos(lc_theta);
                 lc_p2(2)-detection_range*cos(lc_theta);
                 lc_p2(2)+detection_range*cos(lc_theta);
              ];
             tmp_array = [lc_array_x,lc_array_y];
             CT_OUTER_TANGENT_POINTS = [CT_OUTER_TANGENT_POINTS;tmp_array];
         end

end