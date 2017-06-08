% Compare Ground Truth 

home
clear
close all

load ~/datasets/YUDB_GT/DLS_GT.mat
load /home/faraz/datasets/YorkUrbanDB/cameraParameters.mat
focal = focal/pixelSize; 

PLOT = false;

% overall orientation 

benchmark = QUAT_GT_YUDB_ITER;
testList = {...
    'QUAT_GT_YUDB',...
    'QUAT_GT_Relaxed',...
    'QUAT_GT_Relaxed_ITER',...
    'QUAT_GT_NonRelaxed',...
    'QUAT_GT_PCAL',...
    'QUAT_GT_PCAL_ITER'...
    };

legendList = {...
    'YUDB',...
    'GT-ALSx   ',...
    'GT-ALSxIter',...
    'GT-ALS',...
    'GT-PCal',...
    'GT-PCalIter'...
    };

colorList = {...
    'bo-', ...
    'r-', ...    
    'g-', ...    
    'b--', ...
    'k-.', ...    
    'g--', ...
    'b-.', ...    
    'r-.', ...
    'g-.'...
    };

ldw = 2;
mrksz = 3;
fnt = 12;

set(0, 'DefaultAxesFontSize', fnt);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for i = 1:length(testList)
    eval(['test = ' testList{i} ';']);
    devList = zeros(1,102);
    
    for j = 1:102
        dTheta = DiffToSlnFmly(benchmark{j},test{j});
        devList(j) = norm(dTheta)*180/pi;
    end
    
    binSize = .5;
    lastBin = 40;
    theta = 0:binSize:lastBin;
    devHist = histc(devList,theta);
    devCumHist = cumsum(devHist);
    
    figure(1)
    plot(theta+binSize,devCumHist,colorList{i}, 'LineWidth', ldw, 'MarkerSize', mrksz); hold on
end
legend(legendList,'Location','SouthEast')


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
select = [2 4 5]; 
for i = select
    eval(['test = ' testList{i} ';']);
    devList = zeros(1,102);
    
    for j = 1:102
        dTheta = DiffToSlnFmly(benchmark{j},test{j});
        devList(j) = norm(dTheta)*180/pi;
    end
    
    binSize = .1;
    lastBin = 10;
    theta = 0:binSize:lastBin;
    devHist = histc(devList,theta);
    devCumHist = cumsum(devHist);
    
    figure(3)
    plot(theta+binSize,devCumHist,colorList{i},'LineWidth',ldw, 'MarkerSize', mrksz); hold on
end

xlabel('Tilt angle error from GT (deg)');
ylabel('Cumulative number of images');

handle = legend(legendList(select),'Location','SouthEast');

set(gcf, 'PaperPositionMode', 'auto');
axis([0 10 55 105])
set(handle,'Position',[0.64 0.15 0.25 0.1]);

if PLOT
print -depsc2 /home/faraz/tex/VanishingPointsJournal/figs/Compare-ALSx-ALS-PCal.eps
!epstopdf /home/faraz/tex/VanishingPointsJournal/figs/Compare-ALSx-ALS-PCal.eps
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
select = [1 2 4 5]; 
for i = select
    eval(['test = ' testList{i} ';']);
    devList = zeros(1,102);
    
    for j = 1:102
        dTheta = DiffToSlnFmly(benchmark{j},test{j});
        devList(j) = norm(dTheta)*180/pi;
    end
    
    binSize = .5;
    lastBin = 50;
    theta = 0:binSize:lastBin;
    devHist = histc(devList,theta);
    devCumHist = cumsum(devHist);
    
    figure(4)
    plot(theta+binSize,devCumHist,colorList{i},'LineWidth',ldw, 'MarkerSize', mrksz); hold on
end

xlabel('Tilt angle error from GT (deg)');
ylabel('Cumulative number of images');

handle = legend(legendList(select),'Location','SouthEast');

set(gcf, 'PaperPositionMode', 'auto');
axis([0 50 0 105])
set(handle,'Position',[0.64 0.17 0.25 0.1]);

if PLOT
print -depsc2 /home/faraz/tex/VanishingPointsJournal/figs/Compare-ALSx-ALS-PCal-YUDB.eps
!epstopdf /home/faraz/tex/VanishingPointsJournal/figs/Compare-ALSx-ALS-PCal-YUDB.eps
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% focal length
testList = {...
    'foc_GT_PCAL',...
    'foc_GT_PCAL_ITER'...
    };

legendList = {...
    'GT-PCalL',...
    'GT-PCalIter'...
    };


for i = 1:length(testList)
    eval(['test = ' testList{i} ';']);
    devList = zeros(1,102);
    
    for j = 1:102
        devList(j) = abs(test{j}(1) - focal);
    end
    
    binSize = 10;
    lastBin = 200;
    pixel = 0:binSize:lastBin;
    devHist = histc(devList,pixel);
    devCumHist = cumsum(devHist);
    
    figure(2)
    plot(pixel+binSize,devCumHist,colorList{i},'LineWidth',ldw, 'MarkerSize', mrksz); hold on
end
legend(legendList,'Location','SouthEast')
