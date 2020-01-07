
function rt_distance = find_farest_point(landmark,landmarks)
          if(size(landmarks,1)==1 & size(landmarks,2)==2)
             landmarks = [landmarks];
         end
          rt_distance = [];
          for i = 1:size(landmarks,1)
              tmp_lx = landmark(1);
              tmp_ly = landmark(2);
              tmp_object = landmarks(i,:);
              tmp_x = tmp_object(1);
              tmp_y = tmp_object(2);
              tdistance =  sqrt((tmp_lx - tmp_x)^2+(tmp_ly-tmp_y)^2);
              tdistance = tdistance;
              rt_distance = [rt_distance;tdistance];
          end
          rt_distance = norm(rt_distance)*-1;
 end