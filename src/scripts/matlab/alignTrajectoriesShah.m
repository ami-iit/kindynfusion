function [gtBasePosAligned,gtBaseRPYAligned] = alignTrajectoriesShah(gtBasePos, gtBaseRPY, basePos, baseRPY)
assert(length(gtBasePos) == length(basePos), 'vectors must be of same length');
assert(length(gtBaseRPY) == length(baseRPY), 'vectors must be of same length');
assert(length(gtBasePos) == length(gtBaseRPY), 'vectors must be of same length');
assert(length(basePos) == length(baseRPY), 'vectors must be of same length');

nrIters = length(basePos);

gtBasePosAligned = zeros(nrIters, 3);
gtBaseRPYAligned = zeros(nrIters, 3);

AA = zeros(4,4*nrIters);
BB = zeros(4,4*nrIters);
pGnd = iDynTree.Position();
HGnd = iDynTree.Transform();

pEst = iDynTree.Position();
HEst = iDynTree.Transform();

for idx = 1:nrIters
    pGnd.fromMatlab(gtBasePos(idx, :));
    RGnd = iDynTree.Rotation.RPY(gtBaseRPY(idx, 1), ...
                                 gtBaseRPY(idx, 2), ...
                                 gtBaseRPY(idx, 3));
    HGnd.setPosition(pGnd);
    HGnd.setRotation(RGnd);
    BB(:,4*idx-3:4*idx) = HGnd.asHomogeneousTransform().toMatlab();

    pEst.fromMatlab(basePos(idx, :));
    REst = iDynTree.Rotation.RPY(baseRPY(idx, 1), ...
                                 baseRPY(idx, 2), ...
                                 baseRPY(idx, 3));
    HEst.setPosition(pEst);
    HEst.setRotation(REst);
    AA(:,4*idx-3:4*idx) = HEst.asHomogeneousTransform().toMatlab();
end


[PelvisHTracker, EstHGnd] = shah(AA, BB);
HYY = iDynTree.Matrix4x4();
HYY.fromMatlab(EstHGnd);
estHgnd = iDynTree.Transform(HYY);

HXX = iDynTree.Matrix4x4();
HXX.fromMatlab(PelvisHTracker);
trackHPel = iDynTree.Transform(HXX).inverse();

for idx = 1:nrIters
    pGnd.fromMatlab(gtBasePos(idx, :));
    RGnd = iDynTree.Rotation.RPY(gtBaseRPY(idx, 1), ...
                                 gtBaseRPY(idx, 2), ...
                                 gtBaseRPY(idx, 3));
    HGnd.setPosition(pGnd);
    HGnd.setRotation(RGnd);
    Haligned = estHgnd*HGnd*trackHPel;

    gtBasePosAligned(idx, :) = Haligned.getPosition().toMatlab();
    gtBaseRPYAligned(idx, :) = Haligned.getRotation().asRPY().toMatlab();
end

end
