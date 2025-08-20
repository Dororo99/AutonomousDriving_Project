% Please Read Before Run
% How to run
% 1. Run RL_fault_tolerance_Setup.m
% 2. Run main.m
%   Choose "train/test" before run

% 0. use gpu
gpuDevice(1)

% 1. 시스템 열기
startingOpenSystems = find_system('MatchFilter', @Simulink.match.allVariants);

open_system("RL_fault_tolerance_simulation") % 주의) 수동으로 실행 금지!!
pause(2);

% 2. RL 환경 정의
mdl = "RL_fault_tolerance_simulation";
agentblk = mdl + "/Subsystem1/RL Agent";

obsInfo = rlNumericSpec([9 1], ...
    LowerLimit=-inf*ones(9,1), ...
    UpperLimit=inf*ones(9,1));
obsInfo.Name = "observations";
actInfo = rlNumericSpec([2 1], ...
    LowerLimit=[-5;-0.7854], ...
    UpperLimit=[3;0.7854]);
actInfo.Name = "accel;steer";
env = rlSimulinkEnv(mdl,agentblk,obsInfo,actInfo);
rng(0)

% 3. 신경망 및 에이전트 생성
Ts = 0.1; 
Tf = 60;
t_gap = 1.4;
D_default = 10;
v_set = 28;

% 뉴런 개수
L = 128;


% Main path
mainPath = [
    featureInputLayer(prod(obsInfo.Dimension),Name="obsInLyr")
    fullyConnectedLayer(L)
    reluLayer

    fullyConnectedLayer(L)
    reluLayer

    fullyConnectedLayer(L)
    additionLayer(2,Name="add")
    reluLayer
    fullyConnectedLayer(L)
    reluLayer

    fullyConnectedLayer(L)
    reluLayer

    fullyConnectedLayer(1,Name="QValLyr")
    ];
% Action path
actionPath = [
    featureInputLayer(prod(actInfo.Dimension),Name="actInLyr")
    fullyConnectedLayer(L,Name="actOutLyr")
    ];
criticNet = dlnetwork();
criticNet = addLayers(criticNet,mainPath);
criticNet = addLayers(criticNet,actionPath);    
criticNet = connectLayers(criticNet,"actOutLyr","add/in2");
criticNet = initialize(criticNet);
summary(criticNet)
% plot(criticNet)

% create critic 
critic = rlQValueFunction(criticNet,obsInfo,actInfo,...
    ObservationInputNames="obsInLyr",ActionInputNames="actInLyr");
critic.UseDevice = "gpu";

actorNet = [
    featureInputLayer(prod(obsInfo.Dimension))
    fullyConnectedLayer(L)
    reluLayer

    fullyConnectedLayer(L)
    reluLayer
    fullyConnectedLayer(L)
    reluLayer

    fullyConnectedLayer(L)
    reluLayer
    fullyConnectedLayer(L)
    reluLayer
    fullyConnectedLayer(2)
    tanhLayer
    scalingLayer(Scale=[2.5;0.2618],Bias=[-0.5;0])
    ];
actorNet = dlnetwork(actorNet);
actorNet = initialize(actorNet);
summary(actorNet)

% create actor
actor = rlContinuousDeterministicActor(actorNet,obsInfo,actInfo);
actor.UseDevice = "gpu";

gpurng(0) % gpu random seed

criticOptions = rlOptimizerOptions( ...
    LearnRate=1e-4, ...
    GradientThreshold=1, ...
    L2RegularizationFactor=1e-4);

% gpu
% criticOptions.UseDevice = 'gpu';

actorOptions = rlOptimizerOptions( ...
    LearnRate=1e-4, ...
    GradientThreshold=1, ...
    L2RegularizationFactor=1e-4);

% DDPG agent options

agentOptions = rlDDPGAgentOptions(...
    SampleTime=Ts,...
    ActorOptimizerOptions=actorOptions,...
    CriticOptimizerOptions=criticOptions,...
    ExperienceBufferLength=1e6);

% agent variance & std 
agentOptions.NoiseOptions.Variance = [0.8;0.1];
agentOptions.NoiseOptions.VarianceDecayRate = 1e-5;


% 4. 에이전트 선언
agent = rlDDPGAgent(actor,critic,agentOptions);

% 5. 훈련 옵션 및 훈련/로드 결정
maxepisodes = 1e4;
maxsteps = ceil(Tf/Ts);
trainingOpts = rlTrainingOptions(...
    MaxEpisodes=maxepisodes,...
    MaxStepsPerEpisode=maxsteps,...
    Verbose=false,...
    Plots="training-progress",...
    StopTrainingCriteria="EpisodeCount",...
    StopTrainingValue=3000,UseParallel=true);

% 병렬 처리 세부 설정
trainingOpts.ParallelizationOptions.Mode = 'async';
trainingOpts.ParallelizationOptions.DataToSendFromWorkers = 'Gradients';

% doTraining = true; % train
doTraining = false; % test

if doTraining    
    % 에이전트 훈련 (훈련 중 Simulink 모델이 내부적으로 시뮬레이션됨)
    modelName = 'RL_fault_tolerance_simulation';
    save_system(modelName);
    disp(['모델 "', modelName, '" 이(가) 성공적으로 저장되었습니다. 학습을 시작합니다.']);
    trainingStats = train(agent,env,trainingOpts);

    save('trainedDDPGAgent.mat','agent');
    disp('훈련 완료된 agent가 저장되었습니다');
    
    delete(gcp('nocreate')); % 풀 삭제
else
    % 미리 훈련된 에이전트 로드
    load("trainedDDPGAgent.mat","agent")
    disp('Pre-trained agent loaded.');
end

% 6. 시뮬레이션
sim(mdl);