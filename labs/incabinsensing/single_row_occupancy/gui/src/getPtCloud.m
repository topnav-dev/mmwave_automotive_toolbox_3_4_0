function [ptCloud] = getPtCloud(payload)

%   Detailed explanation goes here
ptCloud = struct('numDetectedObj', 0, 'x', [], 'y', [], 'z',[], 'doppler', []);

pointStruct = struct(...
    'x',            {'float', 4}, ... % x, in m
    'y',            {'float', 4}, ... % y, in m
    'z',            {'float', 4}, ... % z, in m
    'doppler',      {'float', 4});    % doppler, in m/s


lengthPointStruct = 4*4;
ptCloud.numDetectedObj = numel(payload)/lengthPointStruct;
if(ptCloud.numDetectedObj)
    p = typecast(uint8(payload),'single');
    p = reshape(p,4, ptCloud.numDetectedObj);

    ptCloud.x = p(1,:);
    ptCloud.y = p(2,:);
    ptCloud.z = p(3,:);
    ptCloud.doppler = p(4,:);
end
end

