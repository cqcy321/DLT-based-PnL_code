function ARGS = JPT_GetArgs()

ARGS = FACADE_ARGS_default();
const = FACADE_const();
ARGS.mKinv = [];

%arguments for Vanishing point detection
ARGS.plot = 0;
ARGS.savePlot = false;

ARGS.manhattanVP = true; %set to false if the focal length is unknown

ARGS.REF_remNonManhantan = true;
ARGS.ALL_calibrated = true;
ARGS.ERROR = const.ERROR_DIST;

%load K for the image of the YDB, we need the principal point to estimate the focal length
load cameraParameters.mat
% focal_GT = focal / pixelSize;
focal = focal / pixelSize;

% FCprintf('Focal length is %f\n', focal);
%this is use normlization, The choice of f is irrelevant if ARGS.manhattanVP=false
ARGS.mK = [[focal,0,pp(1)];[0,focal,pp(2)];[0,0,1]];
ARGS.mKinv = inv(ARGS.mK);