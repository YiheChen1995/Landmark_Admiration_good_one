
function compute_lidar_H_k_new(obj,estimator, params,new_landmarks)
% this funcion builds the Jacobian H for the factor graphs case without
% mesaurements. It uses all the landmarks in the field of view of the lidar

spsi= sin(estimator.x_true(3));
cpsi= cos(estimator.x_true(3));

% number of expected extracted landmarks
obj.LMP_n_L_k= size(new_landmarks,1);

% number of expected measurements
obj.LMP_n_k= obj.LMP_n_L_k * params.m_F;

% build Jacobian
obj.LMP_H_k_new= inf * ones( obj.LMP_n_k , params.m );
for i= 1:obj.LMP_n_L_k
    % Indexes
    indz= 2*i + (-1:0);
    
    dx= new_landmarks(i, 1) - estimator.x_true(1);
    dy= new_landmarks(i, 2) - estimator.x_true(2);
    
    % Jacobian -- H
    obj.LMP_H_k_new (indz,1)= [-cpsi; spsi];
    obj.LMP_H_k_new (indz,2)= [-spsi; -cpsi];
    obj.LMP_H_k_new (indz,params.ind_yaw)= [-dx * spsi + dy * cpsi;
                                   -dx * cpsi - dy * spsi];
                               
end

end