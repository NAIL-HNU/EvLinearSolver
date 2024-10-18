function [ bestHypothesis, RANSAC_inlier_ratio, RANSAC_inlier_id] = RANSAC( inputSet_A, inputSet_b, minSampleSize, threshold, maxSteps)

inputSize = size(inputSet_A, 1);
if nargin < 5
    maxSteps = 3000;
end
bestSize = 0;
bestHypothesis = [];
bestInliersSet = {};
bestOutliersSet = {};
bestGeneratorsSet = [];
bestCrtInliersInd = [];
step = 1;
format long
while step <= maxSteps
    rand_id = ceil(inputSize * rand(minSampleSize, 1));
    A_sample = inputSet_A(rand_id, :);
    b_sample = inputSet_b(rand_id, :);
    sample_solution = A_sample \ b_sample;

    crtErrors = inputSet_A * sample_solution - inputSet_b;

%     r = crtErrors.*crtErrors
    crtInliersInd = crtErrors .* crtErrors < threshold;
    crtSize = sum(crtInliersInd);
    
    if crtSize > bestSize
        bestSize = crtSize;
        bestHypothesis = sample_solution;
    end
    step = step + 1;
    RANSAC_inlier_id = crtInliersInd;
end
RANSAC_inlier_ratio = bestSize/inputSize;
% d = 'end ransac'
end