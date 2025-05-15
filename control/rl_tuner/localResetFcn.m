function in = localResetFcn(in,mdl)

% Randomize reference signal
blk = sprintf([mdl '/Desired \nForce']);
hRef = 10 + 4*(rand-0.5);
in = setBlockParameter(in,blk,'Value',num2str(hRef));

% % Randomize initial water height
% hInit = rand;
% blk = [mdl '/Grinding System/H'];
% in = setBlockParameter(in,blk,'InitialCondition',num2str(hInit));

end