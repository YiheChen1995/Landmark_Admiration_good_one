
function [cst,ceq] = constraint_not_on_road(road_width,x_true,x)

         if(size(x,1)==1 & size(x,2)==2)
             x = [x];
         end
         cst = [];
         px_true = x_true(1);
         py_true = x_true(2);
         heading_true = x_true(3);
         heading_vector = [cos(heading_true),sin(heading_true)];

         T              = [cos(pi/2),-sin(pi/2);
                         sin(pi/2),cos(pi/2)];
         perp_vector = T*heading_vector';

         for i=1:size(x,1)
             x_lm = x(i,:);
             px_lm = x_lm(1);
             py_lm = x_lm(2);
             
             dx = px_lm - px_true;
             dy = py_lm - py_true;
             mm_vector = [dx,dy];
             proj = abs(road_width/2) - abs(dot(mm_vector,perp_vector));
             cst = [cst; proj];
         end
         ceq = [];
end