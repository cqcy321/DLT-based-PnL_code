This is the Matlab code for absolute camera pose estimation from line correspondences using Direct Linear Transformation. The theory and background is described in our paper "Přibyl, Bronislav and Zemčík, Pavel and Čadík, Martin: Absolute Pose Estimation from Line Correspondences using Direct Linear Transformation".

The proposed method DLT-Combined-Lines is implemented in file "DLT_Combined_Lines.m".


=== AVAILABLE TESTS ============================================================

* test_simple5.m
  - A minimal working example.
  - The DLT-Combined-Lines method is run on 5 predefined lines.
  
* test_random.m
  - All methods can be run on randomly generated lines.
  
* test_outliers.m
  - All methods can be run on randomly generated lines with outlying correspondences.
  
* test_VGG.m
  - All methods can be run on real-world image sequences from the VGG multiview dataset.
  - Please uncomment the dataset of your choice at the beginning of the script.
  
* test_MPI.m
  - All methods can be run on real-world image sequences from the MPI datasets for Reconstruction of 3D Line Segment from Images.
  - Please uncomment the dataset of your choice at the beginning of the script.


=== REQUIREMENTS (all of them already included) ================================

* Vanishing Point Toolbox
	- already included
  - VPtoolbox.tgz downloaded from http://www-users.cs.umn.edu/~faraz/?p=research
  - placed into the subfolder functions/VPtoolbox/
  - required to run the method Mirzaei

* RPnL
	- already included
  - RPnLMatlabCode.zip downloaded from http://www.mip.informatik.uni-kiel.de/tiki-index.php?page=Lilian+Zhang
  - placed into the subfolder functions/RPnL/
  - required to run the method RPnL

* Code for: Pose Estimation from Line Correspondences - A Complete Analysis and A Series of Solutions
  - already included
  - code.zip downloaded from http://xuchi.weebly.com/rpnl.html
  - placed into the subfolder functions/PnL_analysis/
  - required to run the methods Ansar, ASPnL, LPnL_Bar_LS, LPnL_Bar_ENull

* MATLAB Functions for Multiple View Geometry (some of them)
  - the 2 functions which are needed are already included
  - downloaded from http://www.robots.ox.ac.uk/~vgg/hzbook/code/
  - placed into the subfolder functions/vgg_multiview/
  - required to run test_VGG.m
  
* VGG Multiview Datasets
  - 7 datasets: Corridor, Merton-College-I, Merton-College-II, Merton-College-III, Model-House, University-Library, Wadham-College
  - all datasets already included (without images which are not necessarry to run our tests)
  - downloaded from http://www.robots.ox.ac.uk/~vgg/data/data-mview.html
  - content of the respective archives unpacked into datasets/VGG/[dataset_name]/ subfolders
  - required to run test_VGG.m
  
* Max-Planck-Institut für Informatik datasets for Reconstruction of 3D Line Segment from Images
  - 3 datasets: building_blocks, street, timber_frame_house
  - all datasets already included (without images which are not necessarry to run our tests)
  - downloaded from http://resources.mpi-inf.mpg.de/LineReconstruction/
  - content of the respective archives unpacked into datasets/MPI/[dataset_name]/ subfolders
  - required to run test_MPI.m
  
