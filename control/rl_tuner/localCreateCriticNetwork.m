function criticNet = localCreateCriticNetwork(numObs, numAct)
% State input path
statePath = [
    featureInputLayer(numObs, Name='stateInLyr')
    fullyConnectedLayer(64, Name='fc1')
    reluLayer(Name='relu1')
    fullyConnectedLayer(128, Name='fc2')
    reluLayer(Name='relu2')
    ];

% Action input path
actionPath = [
    featureInputLayer(numAct, Name='actionInLyr')
    fullyConnectedLayer(64, Name='fc3')
    reluLayer(Name='relu3')
    fullyConnectedLayer(128, Name='fc4')
    reluLayer(Name='relu4')
    ];

% Common path
commonPath = [
    concatenationLayer(1, 2, Name='concat')
    fullyConnectedLayer(128, Name='fcCommon1')
    reluLayer(Name='relu5')
    fullyConnectedLayer(64, Name='fcCommon2')
    reluLayer(Name='relu6')
    fullyConnectedLayer(1, Name='qvalOutLyr')
    ];

% Assemble the critic network
criticNet = dlnetwork();
criticNet = addLayers(criticNet, statePath);
criticNet = addLayers(criticNet, actionPath);
criticNet = addLayers(criticNet, commonPath);

% Connect layers
criticNet = connectLayers(criticNet, 'relu2', 'concat/in1');
criticNet = connectLayers(criticNet, 'relu4', 'concat/in2');
end
