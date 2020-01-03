function [CT_BOUNDARY_POINTS] = define_boundary(CT_OUTER_TANGENT_POINTS)
         
         tmp_odd = [];
         tmp_even = [];
         tmp_merged = [];
         CT_BOUNDARY_POINTS = [];
         
         for i=1:length(CT_OUTER_TANGENT_POINTS)-4

             if(mod(i-1,4)==0)
                lc_x1 =  (CT_OUTER_TANGENT_POINTS(i,:)); %0
                lc_x1 = lc_x1(1);
                lc_y1 =  (CT_OUTER_TANGENT_POINTS(i,:));        %line_a
                lc_y1 = lc_y1(2);
                lc_x2 =  (CT_OUTER_TANGENT_POINTS(i+2,:));%2
                lc_x2 = lc_x2(1);
                lc_y2 =  (CT_OUTER_TANGENT_POINTS(i+2,:));
                lc_y2 = lc_y2(2);
                lc_x3 =  (CT_OUTER_TANGENT_POINTS(i+1,:));%1
                lc_x3 = lc_x3(1);
                lc_y3 =  (CT_OUTER_TANGENT_POINTS(i+1,:));       %line_c
                lc_y3 = lc_y3(2);
                lc_x4 =  (CT_OUTER_TANGENT_POINTS(i+3,:));%3
                lc_x4 = lc_x4(1);
                lc_y4 =  (CT_OUTER_TANGENT_POINTS(i+3,:));
                lc_y4 = lc_y4(2);
                lc_x5 =  (CT_OUTER_TANGENT_POINTS(i+4,:));%4
                lc_x5 = lc_x5(1);
                lc_y5 =  (CT_OUTER_TANGENT_POINTS(i+4,:));       %line_b
                lc_y5 = lc_y5(2);
                lc_x6 =  (CT_OUTER_TANGENT_POINTS(i+6,:));%6
                lc_x6 = lc_x6(1);
                lc_y6 =  (CT_OUTER_TANGENT_POINTS(i+6,:));
                lc_y6 = lc_y6(2);
                lc_x7 =  (CT_OUTER_TANGENT_POINTS(i+5,:));%5
                lc_x7 = lc_x7(1);
                lc_y7 =  (CT_OUTER_TANGENT_POINTS(i+5,:));       %line_d
                lc_y7 = lc_y7(2);
                lc_x8 =  (CT_OUTER_TANGENT_POINTS(i+7,:));%7
                lc_x8 = lc_x8(1);
                lc_y8 =  (CT_OUTER_TANGENT_POINTS(i+7,:)); 
                lc_y8 = lc_y8(2);
              
             
             tmp_x1 = ((lc_x1-lc_x2)*(lc_x5*lc_y6-lc_x6*lc_y5)-(lc_x5-lc_x6)*(lc_x1*lc_y2-lc_x2*lc_y1))/((lc_x5-lc_x6)*(lc_y1-lc_y2)-(lc_x1-lc_x2)*(lc_y5-lc_y6));
             tmp_y1 =((lc_y1-lc_y2)*(lc_x5*lc_y6-lc_x6*lc_y5)-(lc_x1*lc_y2-lc_x2*lc_y1)*(lc_y5-lc_y6))/((lc_y1-lc_y2)*(lc_x5-lc_x6)-(lc_x1-lc_x2)*(lc_y5-lc_y6));
             tmp_x2 = ((lc_x3-lc_x4)*(lc_x7*lc_y8-lc_x8*lc_y7)-(lc_x7-lc_x8)*(lc_x3*lc_y4-lc_x4*lc_y3))/((lc_x7-lc_x8)*(lc_y3-lc_y4)-(lc_x3-lc_x4)*(lc_y7-lc_y8));
             tmp_y2 =((lc_y3-lc_y4)*(lc_x7*lc_y8-lc_x8*lc_y7)-(lc_x3*lc_y4-lc_x4*lc_y3)*(lc_y7-lc_y8))/((lc_y3-lc_y4)*(lc_x7-lc_x8)-(lc_x3-lc_x4)*(lc_y7-lc_y8));
             tmp_array = [tmp_x1,tmp_y1;tmp_x2,tmp_y2];
             
             for j = 1:length(tmp_array)
                 CT_BOUNDARY_POINTS = [CT_BOUNDARY_POINTS;tmp_array(j,:)];
             end
             end
         end
         CT_BOUNDARY_POINTS = [CT_OUTER_TANGENT_POINTS(2,:);CT_BOUNDARY_POINTS];
         CT_BOUNDARY_POINTS = [CT_OUTER_TANGENT_POINTS(1,:);CT_BOUNDARY_POINTS];
         CT_BOUNDARY_POINTS = [CT_BOUNDARY_POINTS;CT_OUTER_TANGENT_POINTS(length(CT_OUTER_TANGENT_POINTS)-1,:)];
         CT_BOUNDARY_POINTS = [CT_BOUNDARY_POINTS;CT_OUTER_TANGENT_POINTS(length(CT_OUTER_TANGENT_POINTS),:)];    
         for i = 1:length(CT_BOUNDARY_POINTS)
             if mod(i,2)==0
                tmp_even = [tmp_even;CT_BOUNDARY_POINTS(i,:)];
             else
                 tmp_odd = [tmp_odd;CT_BOUNDARY_POINTS(i,:)];
             end
         end
         CT_BOUNDARY_POINTS = [tmp_odd;flip(tmp_even,1)];
         
end