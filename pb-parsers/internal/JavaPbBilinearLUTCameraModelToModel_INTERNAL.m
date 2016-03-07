function [ camera_model ] = ...
    JavaPbBilinearLUTCameraModelToModel_INTERNAL( lut_mono )

% load models

lut_left = JavaPbBilinearLUTCameraModelToMatlab(lut_mono(1));
lut_right = JavaPbBilinearLUTCameraModelToMatlab(lut_mono(2));

% set up output
camera_model = struct;
camera_model.baseline = lut_right.G_camerabase_cameraimage(2,4);

camera_model.left = LUTToModel(lut_left);
camera_model.right = LUTToModel(lut_right);

end

function [ model ] = LUTToModel(lut_model)

    model.fx = lut_model.focal_length_x;
    model.fy = lut_model.focal_length_y;
    model.cx = lut_model.principal_point_x;
    model.cy = lut_model.principal_point_y;

    model.lut = lut_model.bilinear_lut_xy;

end