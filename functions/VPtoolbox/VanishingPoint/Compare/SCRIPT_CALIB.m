% RANSAC Run on York Urban Database
%
% $Id: SCRIPT_CALIB.m 1375 2012-07-23 02:47:47Z faraz $

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

kPlotLines = false;
kSaveResults = true;
kMaxHypotheses = 130;
kMinHypotheses = 15;
kInClassResidualThreshold = 0.03;
kMinNumPxlInLine = 40;
kMaxNumLines = 200;
kColinearityThreshold = cos(15*pi/180);

if kSaveResults == true
    out_path = '/tmp/YUDB_Calib_1/';
    if ~exist(out_path,'dir')
        mkdir(out_path);
    end
    fileHandle = fopen(strcat(out_path,'report.csv'),'w+');
else
    fileHandle  = 1;
end

description = {'NoHyst' kInClassResidualThreshold kMinHypotheses};

% varaible allocation
QUAT{102} = zeros(4,1);


%load

for main_counter = 1:102
    disp(main_counter)
    
    % load the image
    name = Manhattan_Image_DB_Names{main_counter}(1:end-1);
    in_path = strcat('/home/faraz/datasets/YorkUrbanDB/',name,'/');
    Ic = imread(strcat(in_path,name,'.jpg'));
    I = rgb2gray(Ic);
    

    [lines_GT dir_GT VP_GT] = LoadGroundTruth_YDB(in_path,name,fc,cc,0);
    N_GT = length(lines_GT);
    q_DLS = EstimateVanishingPoints([lines_GT.nmoment], dir_GT, 'relaxed');
    VP_DLS = quat2rot(q_DLS(:,1))';
    
    
    % get lines from P. Kovesi toolbox
    % if the lines are not cashed, uncomment the following line, and run
    % the script once to cash them. in the following runs use GetLines_PK_Cashed
    % to load the cashed files
    %lines = GetLines_PK(I, kMaxNumLines, in_path);
    lines = GetLines_PK_Cashed(kMaxNumLines, in_path, kMinNumPxlInLine);
    lines = normalize_lines(lines,fc,cc,alpha_c);
    
    
    % uncomment to get lines from J.P. Tardif toolbox 
    %agrs = JPT_GetArgs();
    %[vsEdges, agrs.imE] = FACADE_getEdgelets2(I, agrs);
    %lines = JPT_GetFilteredLines(vsEdges, fc, cc, alpha_c, kMinNumPxlInLine);    
    
    
    [bestClassification, info] = ClassifyLines(...
        [lines.nmoment], ...
        kMaxHypotheses, ...
        kMinHypotheses, ...
        kInClassResidualThreshold, ...
        kColinearityThreshold);    
    bestQuat = info.bestQuat;
    
    QUAT{main_counter} = info.bestQuat;

    % create result images
    if kPlotLines
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
            fc, cc, alpha_c)
        if kSaveResults == true
            print(strcat(out_path,name,'_VP.jpg'),'-djpeg','-r80')
        end
    end
end

if kSaveResults == true
    fclose(fileHandle);
    save(strcat(out_path,'CalibDigest.mat'),'QUAT','description','seed');
end
