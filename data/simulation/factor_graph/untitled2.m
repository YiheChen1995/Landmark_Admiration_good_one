for i =1:12
    idx =  ismember(landmark_map,rm(i,:))
    landmark_map(idx) = []  
end    