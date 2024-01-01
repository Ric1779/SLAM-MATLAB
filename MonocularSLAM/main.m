baseDownloadURL = "https://cvg.cit.tum.de/rgbd/dataset/freiburg3/rgbd_dataset_freiburg3_long_office_household.tgz";
dir             = 'D:\Richards\Projects\Computer Vision\SLAM_MATLAB\MonocularSLAM';
dataFolder      = fullfile(dir, 'tum_rgbd_dataset', filesep);
options         = weboptions(Timeout = Inf);
tgzFileName     = [dataFolder, 'fr3_office.tgz'];
folderExists    = exist(dataFolder, "dir");

% Create a folder in a temporary directory to save the downloaded file
if ~folderExists
    mkdir(dataFolder);
    disp('Downloading fr3_office.tgz (1.38 GB). This download can take a few minutes....')
    websave(tgzFileName, baseDownloadURL, options);
    
    % Extract contents of the downloaded file
    disp('Extracting fr3_office.tgz (1.38) ....')
    untar(tgzFileName, dataFolder);
end

% Use an ImageDatastore object to manage a collection of image files, where each individual
% image fits in memory, but the entire collection of images does not necessarily fit.
imageFolder      = [dataFolder,'rgbd_dataset_freiburg3_long_office_household/rgb/'];
imds             = imageDatastore(imageFolder);

% Inspect the first image
currFrameIdx     = 1;
currI            = readimage(imds, currFrameIdx);
himage           = imshow(currI);

% set random seed for reproducibility
rng(0);

% Creating a cameraIntrinsics object to store the camera intrinsics parameters.
% The images are already undistorted
focalLength      = [535.4, 539.2];         % in units of pixels
principalPoints  = [320.1, 247.6];         % in units of pixels
imageSize        = size(currI, [1,2]);     % in units of pixels
intrinsics       = cameraIntrinsics(focalLength, principalPoints, imageSize);



% Detect and extract ORB features

scaleFactor = 1.2;
numLevels   = 8;
numPoints   = 1000;
[preFeatures, prePoints] = helperDetectAndExtractFeatures(currI, scaleFactor, numLevels, numPoints); 
disp(prePoints);
currFrameIdx = currFrameIdx + 1;
firstI       = currI; % Preserve the first frame 

isMapInitialized  = false;

% Map initialization loop
while ~isMapInitialized && currFrameIdx < numel(imds.Files)
    currI = readimage(imds, currFrameIdx);

    [currFeatures, currPoints] = helperDetectAndExtractFeatures(currI, scaleFactor, numLevels, numPoints); 

    currFrameIdx = currFrameIdx + 1;

    % Find putative feature matches
    indexPairs = matchFeatures(preFeatures, currFeatures, Unique=true, ...
        MaxRatio=0.9, MatchThreshold=40);
    
    % matching method: "Exhaustive (defaut) | "Approximate"
    % "Exhaustive": computer the pairwise distance between feature vectors
    % feature1 and features2
    
    preMatchedPoints  = prePoints(indexPairs(:,1),:);
    currMatchedPoints = currPoints(indexPairs(:,2),:);

    % If not enough matches are found, check the next frame
    minMatches = 100;
    if size(indexPairs, 1) < minMatches
        continue
    end

    preMatchedPoints  = prePoints(indexPairs(:,1),:);
    currMatchedPoints = currPoints(indexPairs(:,2),:);

    % Compute homography and evaluate reconstruction
    [tformH, scoreH, inliersIdxH] = helperComputeHomography(preMatchedPoints, currMatchedPoints);

    % Compute fundamental matrix and evaluate reconstruction
    [tformF, scoreF, inliersIdxF] = helperComputeFundamentalMatrix(preMatchedPoints, currMatchedPoints);

    % Select the model based on a heuristic
    ratio = scoreH/(scoreH + scoreF);
    ratioThreshold = 0.45;
    if ratio > ratioThreshold
        inlierTformIdx = inliersIdxH;
        tform          = tformH;
    else
        inlierTformIdx = inliersIdxF;
        tform          = tformF;
    end

    % Computes the camera location up to scale. Use half of the 
    % points to reduce computation
    inlierPrePoints  = preMatchedPoints(inlierTformIdx);
    inlierCurrPoints = currMatchedPoints(inlierTformIdx);
    [relPose, validFraction] = estrelpose(tform, intrinsics, ...
        inlierPrePoints(1:2:end), inlierCurrPoints(1:2:end));
    
    

    % If not enough inliers are found, move to the next frame
    if validFraction < 0.9 || numel(relPose)==3
        continue
    end

    % Triangulate two views to obtain 3-D map points
    minParallax = 1; % In degrees
    [isValid, xyzWorldPoints, inlierTriangulationIdx] = helperTriangulateTwoFrames(...
        rigidtform3d, relPose, inlierPrePoints, inlierCurrPoints, intrinsics, minParallax);
    
    % tform = rigidtform3d creates a rigidtform3d object
    % that performs an identity transformation.

    if ~isValid
        continue
    end

    % Get the original index of features in the two key frames
    indexPairs = indexPairs(inlierTformIdx(inlierTriangulationIdx),:);

    isMapInitialized = true;
    disp(relPose);
    disp(pose2extr(relPose));
    disp(['Map initialized with frame 1 and frame ', num2str(currFrameIdx-1)])
end 

% End of map initialization loop

if isMapInitialized
    close(himage.Parent.Parent); % Close the previous figure
    % Show matched features
    hfeature = showMatchedFeatures(firstI, currI, prePoints(indexPairs(:,1)), ...
        currPoints(indexPairs(:, 2)), "Montage");
else
    error('Unable to initialize the map.')
end

% Create a an empty imageviewset object to store key frames
vSetKeyFrames = imageviewset;

% The imageviewset object manages view attributes and pairwise connections
% between views of data used in structure-from-motion, visual odometry, and
% simultaneous localization and mapping (SLAM) data. View attributes can be
% feature descriptors, feature points, or absolute camera poses.

% Create an empty worldpointset object to store 3-D map points
mapPointSet = worldpointset;

% The worldpointset object stores correspondences between 3-D world points
% and 2-D image points across camera views. You can use a worldpointset object
% with an imageviewset object to manage image and map data for SfM,
% visual odometry, and SLAM.

% Add the first key-frame. Place the camera associated with the first
% key-frame at the origin, oriented along the z-axis.
preViewId     = 1;
vSetKeyFrames = addView(vSetKeyFrames, preViewId, rigidtform3d, Points=prePoints,...
                Features=preFeatures.Features);

% Add the second key-frame
currViewId    = 2;
vSetKeyFrames = addView(vSetKeyFrames, currViewId, relPose, Points=currPoints,...
                Features=currFeatures.Features);

% Add connection between the first and the second key-frame
vSetKeyFrames = addConnection(vSetKeyFrames, preViewId, currViewId, relPose, ...
                Matches=indexPairs);

% Add 3-D map points
[mapPointSet, newPointIdx] = addWorldPoints(mapPointSet, xyzWorldPoints);

% Add observations of the map points
preLocations  = prePoints.Location;
currLocations = currPoints.Location;
preScales     = prePoints.Scale;
currScales    = currPoints.Scale;

% Add image points corresponding to the map points in the first key-frame
mapPointSet = addCorrespondences(mapPointSet, preViewId, newPointIdx, indexPairs(:,1));
% in the frame 'preViewId' the world points 'newPointIdx' corresponds to
% the 'indexPairs' location in the frame

% Add image points corresponding to the map points in the second key-frame
mapPointSet = addCorrespondences(mapPointSet, currViewId, newPointIdx, indexPairs(:,2));

% BoW INITIALIZATION FOR LOOP CLOSURE

% Load the bag of features data created offline
bofData         = load("bagOfFeaturesDataSLAM.mat");
% bofData = bagOfFeatures(imds,CustomExtractor=@helperORBFeatureExtractorFunction,...
%     TreeProperties=[3, 10],StrongestFeatures=1);

% Initialize the place recognition database
loopDatabase    = invertedImageIndex(bofData.bof,SaveFeatureLocations=false);

% Add features of the first two key frames to the database
addImageFeatures(loopDatabase, preFeatures, preViewId);
addImageFeatures(loopDatabase, currFeatures, currViewId);

