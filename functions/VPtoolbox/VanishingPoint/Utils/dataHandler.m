% dataHandler.m

load /home/faraz/datasets/YUDB_PartCalib_1/FocDigest.mat

load /home/faraz/datasets/YorkUrbanDB/Manhattan_Image_DB_Names.mat
load /home/faraz/datasets/YorkUrbanDB/cameraParameters.mat
fc = [1;1]*focal/pixelSize; cc = pp'; alpha_c = 0;

for j = 1:102
    
    % load the image
    name = Manhattan_Image_DB_Names{j}(1:end-1);
    in_path = strcat('/home/faraz/datasets/YorkUrbanDB/',name,'/');
    Ic = imread(strcat(in_path,name,'.jpg'));
    I = rgb2gray(Ic);
    

    [lines_GT dir_GT VP_GT] = LoadGroundTruth_YDB(in_path,name,fc,cc,0);
    N_GT = length(lines_GT);
    q_DLS = EstimateVanishingPoints([lines_GT.nmoment], dir_GT, 'relaxed');
    VP_DLS = quat2rot(q_DLS(:,1))'; 
    
    VP_JP = quat2rot(bestQuat{j})';
    QUAT{j} = bestQuat{j};
    
    crss(1,:) = VP_JP(:,1)'*VP_DLS;
    crss(2,:) = VP_JP(:,2)'*VP_DLS;
    crss(3,:) = VP_JP(:,3)'*VP_DLS;
    diff_RNS = sort(min(acos(abs(crss))*180/pi));
    
    diff_RNS_min(j) = diff_RNS(1);
    diff_RNS_max(j) = diff_RNS(3);
    
end

save('/home/faraz/datasets/YUDB_PartCalib_1/CalibDigest.mat','QUAT','diff_RNS_min','diff_RNS_max');