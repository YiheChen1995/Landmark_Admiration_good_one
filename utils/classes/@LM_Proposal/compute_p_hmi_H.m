
function LMP_p_hmi_H= compute_p_hmi_H(obj, alpha, fault_ind, params)

 
% build fault-free msmts extraction matrix
if fault_ind == 0
    obj.compute_B_matrix_fg( [] , params.m_F );
else
    obj.compute_B_matrix_fg( fault_ind , params.m_F );
end

LMP_Lambda_h_j= obj.A_aug' * obj.B_j' * obj.B_j * obj.A_aug;

LMP_sigma_hat_j = sqrt( alpha' / LMP_Lambda_h_j * alpha );

LMP_sigma_hat_delta_j= sqrt( LMP_sigma_hat_j^2 - obj.LMP_sigma_hat^2 );

LMP_T_delta_j= norminv( 1 - obj.C_req/( 2*(obj.LMP_n_H+1) ) ) * LMP_sigma_hat_delta_j;

% check if faults can be monitored for the given fault hypothesis
if ( ( (params.alert_limit-LMP_T_delta_j)>=0 )...
   && isreal(LMP_sigma_hat_j)...
   && isreal(LMP_T_delta_j) )
    LMP_p_hmi_H= 2*normcdf( -1*(params.alert_limit-LMP_T_delta_j), 0, LMP_sigma_hat_j );
else
    LMP_p_hmi_H= 1;
end

end
