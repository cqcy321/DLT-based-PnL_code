% Get Ground Truth on York Urban Database
%
% $Id: SCRIPT_DLS_GT.m 1342 2012-02-13 07:32:10Z faraz $

disp('------------------------------------------------------')
clear
close all    

ttt = clock; seed = fix(ttt(6)*1e6)
% seed = 42359286
rand('twister',seed);
randn('state',seed);

load /home/faraz/datasets/YorkUrbanDB/Manhattan_Image_DB_Names.mat
load /home/faraz/datasets/YorkUrbanDB/cameraParameters.mat
fc = [1;1]*focal/pixelSize; cc = pp'; alpha_c = 0;

% options
kSaveResults = false;
kPlotLines = false;
approxFocal = 600*ones(2,1);

% where to save 
if kSaveResults == true
    out_path = '/home/faraz/datasets/YUDB_GT/';
    if ~exist(out_path,'dir')
        mkdir(out_path);
    end
    fileHandle = fopen(strcat(out_path,'report.csv'),'w+');
else
    fileHandle  = 1;
end

% variable allocation 
QUAT_GT_YUDB{102} = zeros(4,1);
QUAT_GT_Relaxed{102} = zeros(4,1);
QUAT_GT_NonRelaxed{102} = zeros(4,1);
QUAT_GT_YUDB_ITER{102} = zeros(4,1);
QUAT_GT_Relaxed_ITER{102} = zeros(4,1);
QUAT_GT_PCAL{102} = zeros(4,1);
foc_GT_PCAL{102} = zeros(2,1);
QUAT_GT_PCAL_ITER{102} = zeros(4,1);
foc_GT_PCAL_ITER{102} = zeros(2,1);

% main loop
for main_counter = 2%:102
    disp(main_counter)
    
    % load the image
    name = Manhattan_Image_DB_Names{main_counter}(1:end-1);
    in_path = strcat('/home/faraz/datasets/YorkUrbanDB/',name,'/');
    Ic = imread(strcat(in_path,name,'.jpg'));
    I = rgb2gray(Ic);
    

    % load ground truth lines and vanishing points from YUD
    [lines_GT dir_GT VP_GT] = LoadGroundTruth_YDB(in_path,name,fc,cc,alpha_c);
    N_GT = length(lines_GT);

    
    % save the ground truth given by YUDB
    if any(det(VP_GT) < 0),
        [VP_GT ~] = qr(VP_GT);
    end
    QUAT_GT_YUDB{main_counter} = rot2quat(VP_GT);    
    
    
    % iterative refinement of YUDB GT VP
    q = GetVPIterLS(rot2quat(VP_GT), [lines_GT.nmoment], dir_GT);
    QUAT_GT_YUDB_ITER{main_counter} = q;
    
    
    % save the ground truth given by our relaxed method
    q = EstimateVanishingPoints([lines_GT.nmoment], dir_GT, 'relaxed');
    QUAT_GT_Relaxed{main_counter} = q(:,1);  
    
    
    % iterative refinement of YUDB GT VP
    q = GetVPIterLS(q(:,1), [lines_GT.nmoment], dir_GT);
    QUAT_GT_Relaxed_ITER{main_counter} = q;

    
    % save the ground truth given by our nonrelaxed method
    opts = EstimateVanishingPointsDefaultOpts('nonrelaxed');
    opts.NeMax = opts.NeMax + 3;
    q = EstimateVanishingPoints([lines_GT.nmoment], dir_GT, 'nonrelaxed', opts);
    QUAT_GT_NonRelaxed{main_counter} = q(:,1);
    
    
    % re-normalize lines with approx. focal length
    lines = normalize_lines(lines_GT,approxFocal,cc,0);
    % DLS (using ground truth line association)
    [q foc] = EstimateVanishingPointsFoc([lines.nmoment], dir_GT, 'relaxed');
    foc_GT_PCAL{main_counter} = approxFocal./foc(1);
    QUAT_GT_PCAL{main_counter} = q(:,1);
    
    
    % iterative refinement of GT PCAL VP and Foc
    [q foc] = GetVPFocIterLS(q(:,1), foc(1), [lines.nmoment], dir_GT);
    foc_GT_PCAL_ITER{main_counter} = approxFocal./foc;
    QUAT_GT_PCAL_ITER{main_counter} = q;
    
    
    % create result images
    if kPlotLines
        close all;
        plot_line_segments(I,lines);
        if kSaveResults == true
            print(strcat(out_path,name,'_LINES.jpg'),'-djpeg','-r80')
        end
        plot_line_segments(I,lines, bestClassification);
        if kSaveResults == true
            print(strcat(out_path,name,'_CLASSES.jpg'),'-djpeg','-r80')
        end        
        plot_vanishing_points(I, ...
            lines(bestClassification ~= 0), ...
            bestClassification(bestClassification ~= 0), ...
            quat2rot(bestQuat)', ...
            bestFoc, cc, alpha_c)
        if kSaveResults == true
            print(strcat(out_path,name,'_VP.jpg'),'-djpeg','-r80')
        end
    end
    
end

if kSaveResults == true
    fclose(fileHandle);
    save(strcat(out_path,'DLS_GT.mat'), ...
    'QUAT_GT_YUDB', ...
    'QUAT_GT_YUDB_ITER', ...
    'QUAT_GT_Relaxed', ...
    'QUAT_GT_Relaxed_ITER', ...
    'QUAT_GT_NonRelaxed', ...
    'QUAT_GT_PCAL', ...
    'foc_GT_PCAL', ...
    'QUAT_GT_PCAL_ITER', ...
    'foc_GT_PCAL_ITER');
end
