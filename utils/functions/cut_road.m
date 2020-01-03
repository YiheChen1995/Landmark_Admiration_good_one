function [CT_BOUNDARY_POINTS,CT_OUTER_TANGENT_POINTS] = cut_road(MN_ROUTE,detection_range)

         CT_DEBUG = 0;
         CT_THETAS = [];
         CT_ROUTE = MN_ROUTE;
         CT_BORDER_POINTS = [];
         CT_OUTER_TANGENT_POINTS = [];
         CT_BOUNDARY_POINTS = [];
         
         [CT_THETAS,CT_OUTER_TANGENT_POINTS] = dema_tangent_points(CT_ROUTE,detection_range);
         [CT_BOUNDARY_POINTS] = define_boundary(CT_OUTER_TANGENT_POINTS);
end

