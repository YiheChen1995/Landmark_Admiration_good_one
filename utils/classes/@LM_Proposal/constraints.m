function [cst,ceq] = constraints(obj,im,params,estimator,x_true,new_landmarks)

[cst,ceq] = constraint_in_detection_range(params,x_true,new_landmarks);
cst=[cst;(1e8)*(obj.calculate_phmi_pseudo(im,params,estimator,new_landmarks)/1e8-(1e-7))];
[cst1,ceq] =  constraint_not_on_road(15,x_true,new_landmarks);
cst = [cst;cst1];

end