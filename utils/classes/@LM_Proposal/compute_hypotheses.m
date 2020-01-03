
function compute_hypotheses(obj, params)
    
new_H= nchoosek(obj.LMP_n_L_M, 1);
obj.LMP_inds_H= ones(new_H,1);
start_ind= 1;
obj.LMP_inds_H( start_ind:start_ind+new_H - 1, 1 )= nchoosek(1:obj.LMP_n_L_M, 1);

end
