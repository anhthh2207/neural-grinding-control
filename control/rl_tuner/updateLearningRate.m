function stop = updateLearningRate(info)
    stop = false;  % Do not stop training
    
    % At decay episode, update learning rates
    if info.EpisodeIndex == 300
        % Drop learning rate
        newLR = 1e-6;

        % Get agent from base workspace
        agent = evalin('base', 'agent');

        % Modify optimizer options
        agent.AgentOptions.ActorOptimizerOptions.LearnRate = newLR;
        agent.AgentOptions.CriticOptimizerOptions.LearnRate = newLR;

        % Push updated agent back to base workspace
        assignin('base', 'agent', agent);

        disp('Learning rate dropped at episode 300!');
    end
end