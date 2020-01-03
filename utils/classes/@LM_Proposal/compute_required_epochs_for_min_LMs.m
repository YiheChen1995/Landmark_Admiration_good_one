function compute_required_epochs_for_min_LMs(obj, params, estimator,new_landmarks)

obj.LMP_n_M= estimator.n_k+size(new_landmarks,1)*params.m_F;
i= 0; % initialize i to zero to indicate the current epoch
if sum(obj.n_ph) ~= 0
    for i= 1:length(obj.n_ph)
        obj.LMP_n_M= obj.LMP_n_M + obj.n_ph(i);
        % if the preceding horizon is long enough --> stop
        if ((obj.LMP_n_M/params.m_F)  >= params.min_n_L_M ), break, end
    end
end
% set the variables
obj.LMP_n_L_M= obj.LMP_n_M / params.m_F;
estimator.LMP_n_L_M= obj.LMP_n_L_M;
obj.M= i + 1; % preceding epochs plus the current

end