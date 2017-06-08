% Compare Ground Truth

% Opt.Format = 'base64';
% newQuatName = ['QUAT_' DataHash(description,Opt)];
% newDescName = ['DESC_' DataHash(description,Opt)];
% eval([newQuatName '= QUAT_RNS_HYST;'])
% eval([newDescName '= description;'])
%
% if kSaveResults == true
%     fclose(fileHandle);
%     save(strcat(out_path,'CalibDigest.mat'),newQuatName,newDescName);
% end

home
clear
close all

PLOT = true;

% load camera parameters
load /home/faraz/datasets/YorkUrbanDB/cameraParameters.mat
focal = focal/pixelSize;

% load benchmark
load /home/faraz/datasets/YUDB_GT/DLS_GT.mat
benchmark = QUAT_GT_Relaxed;
%benchmark = QUAT_GT_YUDB_ITER;


% calibration with no hysteresis 
dataset(1).path = '~/datasets/YUDB_Calib_0/CalibDigest.mat';
dataset(1).desc = 'PK 40 0.03 15';
dataset(1).color = 'c';

dataset(2).path = '~/datasets/YUDB_Calib_1/CalibDigest.mat';
dataset(2).desc = 'PK 40 0.03 25';
dataset(2).color = 'c';

dataset(3).path = '~/datasets/YUDB_Calib_2/CalibDigest.mat';
dataset(3).desc = 'RNS';
dataset(3).color = 'c';



% calibration with hysteresis 
dataset(21).path = '~/datasets/YUDB_Calib_Hyst_0/CalibDigest.mat';
dataset(21).desc = 'hRNS';
dataset(21).color = 'b--';

dataset(22).path = '~/datasets/YUDB_Calib_Hyst_1/CalibDigest.mat';
dataset(22).desc = 'PK 40 0.02 0.08 Hyst 25';
dataset(22).color = 'k';

dataset(23).path = '~/datasets/YUDB_Calib_Hyst_2/CalibDigest.mat';
dataset(23).desc = 'PK 40 0.02 0.08 Hyst 25';
dataset(23).color = 'k';

dataset(24).path = '~/datasets/YUDB_Calib_Hyst_3/CalibDigest.mat';
dataset(24).desc = 'JP 40 0.02 0.08 Hyst 25';
dataset(24).desc = 'RNS'; % print legend
dataset(24).color = 'b';

dataset(25).path = '~/datasets/YUDB_Calib_Hyst_4/CalibDigest.mat';
dataset(25).desc = 'JP 40 0.02 0.08 Hyst 25';
dataset(25).color = 'k';


% JPT
dataset(31).path = '~/datasets/YUDB_Calib_JPT_0/CalibDigest.mat';
dataset(31).desc = 'JPT';
dataset(31).color = 'r--';

dataset(32).path = '~/datasets/YUDB_Calib_JPT_1/CalibDigest.mat';
dataset(32).desc = 'JPT';
dataset(32).color = 'r-.';

dataset(33).path = '~/datasets/YUDB_Calib_JPT_2/CalibDigest.mat';
dataset(33).desc = 'JPT';
dataset(33).color = 'r--';

dataset(34).path = '~/datasets/YUDB_Calib_JPT_2/CalibDigest.mat';
dataset(34).desc = 'JPT NoPick';
dataset(34).color = 'r-';


% EM
dataset(41).path = '~/datasets/YUDB_Calib_EM_0/CalibDigest.mat';
dataset(41).desc = 'EM';
dataset(41).color = 'g-d';

dataset(42).path = '~/datasets/YUDB_Calib_EM_2/CalibDigest.mat';
dataset(42).desc = 'EM';
dataset(42).color = 'go-';

dataset(43).path = '~/datasets/YUDB_Calib_EM_DH_0/CalibDigest.mat';
dataset(43).desc = 'EM DH';
dataset(43).color = 'k';

dataset(44).path = '~/datasets/YUDB_Calib_EM_DH_1/CalibDigest.mat';
dataset(44).desc = 'EM';
dataset(44).color = 'gd-';


dataset(51).path = '~/datasets/YUDB_Calib_EM_1/CalibDigest.mat';
dataset(51).desc = 'Li';
dataset(51).color = 'k';


ldw = 2;
mrksz = 6;
fnt = 12;

set(0, 'DefaultAxesFontSize', fnt);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
select = [3 21 32 44];


for i = select
    
    clear QUAT
    load(dataset(i).path);
    devList = zeros(1,102);
    
    for j = 1:102
        if ~isempty(QUAT{j})
            dTheta = DiffToSlnFmly(benchmark{j},QUAT{j});
            devList(j) = norm(dTheta)*180/pi;
        else
            devList(j) = 90;
        end
    end
    
    binSize = .5;
    lastBin = 20;
    theta = 0:binSize:lastBin;
    devHist = histc(devList,theta);
    devCumHist = cumsum(devHist);
    
    figure(1)
    plot(theta+binSize,devCumHist,dataset(i).color,'LineWidth',ldw, 'MarkerSize', mrksz); hold on
end

xlabel('Tilt angle error from GT (deg)');
ylabel('Cumulative number of images');

handle = legend(dataset(select).desc,'Location','SouthEast');

set(gcf, 'PaperPositionMode', 'auto');
axis([0 11 0 105])
set(handle,'Position',[0.64 0.17 0.25 0.1]);

if PLOT
    saveas(gcf, '/home/faraz/tex/VanishingPointsJournal/figs/Tilt-Comp.fig')
    print -depsc2 /home/faraz/tex/VanishingPointsJournal/figs/Tilt-Comp.eps
    !epstopdf /home/faraz/tex/VanishingPointsJournal/figs/Tilt-Comp.eps
end

return

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%select = [1:3];


for i = select
    
    clear QUAT    
    load(dataset(i).path);    
    diff_TEST_min = zeros(1,102);
    diff_TEST_max = zeros(1,102);    
    
    for j = 1:102
        if ~isempty(QUAT{j})
            
            VP_BEN = quat2rot(benchmark{j})';
            
            VP_TEST = quat2rot(QUAT{j})';
            
            %crss(1,:) = VP_TEST(:,1)'*VP_BEN;
            %crss(2,:) = VP_TEST(:,2)'*VP_BEN;
            %crss(3,:) = VP_TEST(:,3)'*VP_BEN;
            crss = VP_TEST'*VP_BEN;
            
            diff_TEST = sort(min(acos(abs(crss))*180/pi));
            
            diff_TEST_min(j) = diff_TEST(1);
            diff_TEST_max(j) = diff_TEST(3);
            
        else
            diff_TEST_min(j) = 90;
            diff_TEST_max(j) = 90;
        end
    end
    
    binSize = .5;
    lastBin = 10;
    theta = 0:binSize:lastBin;
    
    %devHist = histc(devList,theta);
    %devCumHist = cumsum(devHist);
    
    elem_TEST_max = histc(diff_TEST_max,theta);
    elem_TEST_min = histc(diff_TEST_min,theta);
    
    cum_TEST_max = cumsum(elem_TEST_max);
    cum_TEST_min = cumsum(elem_TEST_min);
    
    figure(2)
    plot(theta+binSize,cum_TEST_max,dataset(i).color,'LineWidth',ldw, 'MarkerSize', mrksz); hold on
    
    figure(3)
    plot(theta+binSize,cum_TEST_min,dataset(i).color,'LineWidth',ldw, 'MarkerSize', mrksz); hold on
    
end


%%%%%%%%%%%%% Load existing datasets

% dataset(100).path = '~/datasets/YorkUrbanDB/DevDigest.mat';
% dataset(100).desc = 'ICCV';
% dataset(100).color = 'g';
% 
% select = [select 100];
% 
% for cntr = 100
%     load(dataset(cntr).path);
%     
%     elem_TEST_max = histc(diff_RNS_max,theta);
%     elem_TEST_min = histc(diff_RNS_min,theta);
%     
%     cum_TEST_max = cumsum(elem_TEST_max);
%     cum_TEST_min = cumsum(elem_TEST_min);
%     
%     figure(2)
%     plot(theta+binSize,cum_TEST_max,'g-.','LineWidth',ldw, 'MarkerSize', mrksz); hold on
%     
%     figure(3)
%     plot(theta+binSize,cum_TEST_min,'g-.','LineWidth',ldw, 'MarkerSize', mrksz); hold on
% end

%%%%%%%%%%%


figure(2)
%title('max deviation (deg)')
%legend(dataset(select).desc,'Location','SouthEast')

xlabel('Max deviation from GT (deg)');
ylabel('Cumulative number of images');

handle = legend(dataset(select).desc,'Location','SouthEast');
% 
% set(gcf, 'PaperPositionMode', 'auto');
axis([0 11 0 105])
set(handle,'Position',[0.64 0.17 0.25 0.1]);

if PLOT
    print -depsc2 /home/faraz/tex/VanishingPointsJournal/figs/CHANGENAME.eps
    !epstopdf /home/faraz/tex/VanishingPointsJournal/figs/CHANGENAME.eps
end

figure(3)
title('min deviation (deg)')
legend(dataset(select).desc,'Location','SouthEast')


if PLOT
    print -depsc2 /home/faraz/tex/VanishingPointsJournal/figs/CHANGENAME.eps
    !epstopdf /home/faraz/tex/VanishingPointsJournal/figs/CHANGENAME.eps
end




