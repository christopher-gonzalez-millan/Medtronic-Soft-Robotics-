%% SoftRobot Test
% This file will attempt to calculate the 3d positions of a soft robot

% robot width from inner diameter to outer diameter, when curved this is
% they delta in radius
dR = .1;

% Inner segment arc length - this is the length of each bubble segment 
% when not expanded.  The inner length will curve, the outer segment is 
% what gets expanded
Cinner = 1;
Couter = 1;

% Number of segments making up the soft robot
numSeg = 10;
% length of each bubble segment
SegLen = 1; 


%% Data Generation for Training
% Use Neural Net Fitting App - Input=dIn, Output= dOut, column vectors 
% 25 hidden layers, baysian or leveberg training 

% Delta Circumfrence Outer expansion - this is how much the outer arc expands, thus causing 
% curvature
dCouter_0 = 0.064;
dCouter_120 = 0.009;
dCouter_240 = 0.0;

% PreAllocate
rng = 0.03;
step = 0.0005;
numSteps = (rng*2)/step;
total = numSteps^3;
v = zeros(total,3);
dIn = zeros(total,3);
dOut = zeros(total,3);
ds = zeros(total,6);

j = 1; % dataset counter

for dCouter_0 = -rng:step:rng
    disp(dCouter_0);
    for dCouter_120 = -rng:step:rng
        for dCouter_240 = -rng:step:rng
            % Calculate the bend radius of the circle and angle theta of a single
            % arc formed when the outter bubbles expand (dC) for each bubble set
            % at 0,120,240 deg from the x axis

            [theta0,R0] = Calc_R_Theta(Cinner,dCouter_0,dR);
            [theta120,R120] = Calc_R_Theta(Cinner,dCouter_120,dR);
            [theta240,R240] = Calc_R_Theta(Cinner,dCouter_240,dR);

            % Calculate the bend angle (angle of rotation of the counter clockwise
            % toward the y axis)
            [BendAngle0] = CalcBendAngle(theta0,R0);
            [BendAngle120] = CalcBendAngle(theta120,R120);
            [BendAngle240] = CalcBendAngle(theta240,R240);

            [v] = CalcPosition(BendAngle0,BendAngle120,BendAngle240,SegLen,numSeg);

            dIn(j,:) = [v(end,:)];
            dOut(j,:) = [dCouter_0,dCouter_120,dCouter_240];
            ds(j,:) = [dCouter_0,dCouter_120,dCouter_240,v(end,:)];
            j=j+1;
            
            
        end
    end
end

%% Normalize datasets
% not sure this is needed
if true == 0
    dIn_max = max(dIn,[],'all');
    dOut_max = max(dOut,[],'all');
    dIn_N = dIn./dIn_max;
    dOut_N = dOut./dOut_max;
end

%% Train NN

%trainFcn = 'trainlm';  % Levenberg-Marquardt backpropagation.
%trainFcn = 'trainbr';  % Bayesian Regularization
trainFcn = 'trainscg';  % Scaled Conjugate Gradient
%trainFcn = 'traingdx';  % Varialbe Learning Rate Back Prop

% Create a Fitting Network
hiddenLayer1Size = 25;
hiddenLayer2Size = 25;
hiddenLayer3Size = 0;

%net = fitnet([hiddenLayer1Size hiddenLayer2Size hiddenLayer3Size],trainFcn);
net = fitnet([hiddenLayer1Size hiddenLayer2Size],trainFcn);
%net = fitnet([hiddenLayer1Size],trainFcn);
net = train(net,dIn',dOut');

%% This creates a function for above network
genFunction(net,'CalcBubbleVals');

%% Plot training data set

[len,xx] = size(dIn);
step = 1000;
hold off

for i=1:step:len
    % Calculate the bend radius of the circle and angle theta of a single
    % arc formed when the outter bubbles expand (dC) for each bubble set
    % at 0,120,240 deg from the x axis

    dCouter_0 = dOut(i,1);
    dCouter_120 = dOut(i,2);
    dCouter_240 = dOut(i,3);
    
    [theta0,R0] = Calc_R_Theta(Cinner,dCouter_0,dR);
    [theta120,R120] = Calc_R_Theta(Cinner,dCouter_120,dR);
    [theta240,R240] = Calc_R_Theta(Cinner,dCouter_240,dR);

    % Calculate the bend angle (angle of rotation of the counter clockwise
    % toward the y axis)
    [BendAngle0] = CalcBendAngle(theta0,R0);
    [BendAngle120] = CalcBendAngle(theta120,R120);
    [BendAngle240] = CalcBendAngle(theta240,R240);

    [v] = CalcPosition(BendAngle0,BendAngle120,BendAngle240,SegLen,numSeg);

    % Plot results
    plot3(v(:,1),v(:,2),v(:,3),'-r');
    
    if i == 1
        hold on
    end
    
end

% Setup Plot
xlabel('x')
ylabel('y')
zlabel('z')
axisLim = numSeg*SegLen;
%axisLim = 1;
axis([-axisLim axisLim -axisLim axisLim -axisLim axisLim])
title('Training Set Plot');
legend('Act');

grid on



%% Plot NN results 

[len,xx] = size(dIn);
step = 1000;
hold off


for i=1:step:len
    % Calculate the bend radius of the circle and angle theta of a single
    % arc formed when the outter bubbles expand (dC) for each bubble set
    % at 0,120,240 deg from the x axis

    % Plot actual robot path
    if true == 1
        dCouter_0 = dOut(i,1);
        dCouter_120 = dOut(i,2);
        dCouter_240 = dOut(i,3);

        [theta0,R0] = Calc_R_Theta(Cinner,dCouter_0,dR);
        [theta120,R120] = Calc_R_Theta(Cinner,dCouter_120,dR);
        [theta240,R240] = Calc_R_Theta(Cinner,dCouter_240,dR);

        % Calculate the bend angle (angle of rotation of the counter clockwise
        % toward the y axis)
        [BendAngle0] = CalcBendAngle(theta0,R0);
        [BendAngle120] = CalcBendAngle(theta120,R120);
        [BendAngle240] = CalcBendAngle(theta240,R240);

        [vAct] = CalcPosition(BendAngle0,BendAngle120,BendAngle240,SegLen,numSeg);
        % Plot results
        plot3(vAct(:,1),vAct(:,2),vAct(:,3),'-r');
        
    end

    % NN results
    [Y,Xf,Af] = CalcBubbleVals(dIn(i,:)');    
    
    % Plot point cloud only
    if true == 0

        scatter3(dOut(i,1),dOut(i,2),dOut(i,3),'MarkerEdgeColor','r');
        scatter3(Y(1),Y(2),Y(3),'MarkerEdgeColor','b');
    
    end
    
    % Plot NN full robot path
    if true == 1
        dCouter_0 = Y(1);
        dCouter_120 = Y(2);
        dCouter_240 = Y(3);

        [theta0,R0] = Calc_R_Theta(Cinner,dCouter_0,dR);
        [theta120,R120] = Calc_R_Theta(Cinner,dCouter_120,dR);
        [theta240,R240] = Calc_R_Theta(Cinner,dCouter_240,dR);

        % Calculate the bend angle (angle of rotation of the counter clockwise
        % toward the y axis)
        [BendAngle0] = CalcBendAngle(theta0,R0);
        [BendAngle120] = CalcBendAngle(theta120,R120);
        [BendAngle240] = CalcBendAngle(theta240,R240);

        [vNN] = CalcPosition(BendAngle0,BendAngle120,BendAngle240,SegLen,numSeg);

        % Plot results
        plot3(vNN(:,1),vNN(:,2),vNN(:,3),'-b');
        
    end

    if i == 1
        hold on
    end
    
    
end

% Setup Plot
xlabel('x')
ylabel('y')
zlabel('z')
axisLim = numSeg*SegLen;
axis([-axisLim axisLim -axisLim axisLim -axisLim axisLim])
title('NN Prediction Results');
legend('Pred','Act');

grid on


%% Plot NN Error

[len,xx] = size(dIn);
errMax = 0.003;
step = 1000;

% PreAllocate
vErr = zeros(len,3);
eLen = zeros(len,1);

% Plot Control
PointCloud = true;

hold off

for i=1:step:len

    % NN results
    [Y,Xf,Af] = CalcBubbleVals(dIn(i,:)');    

    % Calculate NN path
    dCouter_0 = Y(1);
    dCouter_120 = Y(2);
    dCouter_240 = Y(3);

    [theta0,R0] = Calc_R_Theta(Cinner,dCouter_0,dR);
    [theta120,R120] = Calc_R_Theta(Cinner,dCouter_120,dR);
    [theta240,R240] = Calc_R_Theta(Cinner,dCouter_240,dR);

    % Calculate the bend angle (angle of rotation of the counter clockwise
    % toward the y axis)
    [BendAngle0] = CalcBendAngle(theta0,R0);
    [BendAngle120] = CalcBendAngle(theta120,R120);
    [BendAngle240] = CalcBendAngle(theta240,R240);

    [vNN] = CalcPosition(BendAngle0,BendAngle120,BendAngle240,SegLen,numSeg);

    % Calculate error
    % Vector error
    vErr(i,:) = vNN(end,:)- dIn(i,:);
    % vector error length
    eLen(i) = sqrt(vErr(i,1)^2 + vErr(i,2)^2 + vErr(i,3)^2);
    
    err = sum(abs(vNN(end,:)- dIn(i,:)));
    % attempt to normalize
    if err > errMax
        errMax = err;
    end
    err = err/errMax;
        
    % Point cloud direct compare
    if PointCloud == true
        scatter3(dIn(i,1),dIn(i,2),dIn(i,3),...
            'MarkerEdgeColor','k',...
            'MarkerFaceColor',[0 0 0]);

        scatter3(vNN(end,1),vNN(end,2),vNN(end,3),...
            'MarkerEdgeColor','b',...
            'MarkerFaceColor',[eLen(i) 1-eLen(i) 1-eLen(i)]);
    end

    % Vector error
    if PointCloud == false
        scatter3(vErr(i,1),vErr(i,2),vErr(i,3),...
            'MarkerEdgeColor','b',...
            'MarkerFaceColor',[eLen(i) 1-eLen(i) 1-eLen(i)]);
    end

    if i == 1
        hold on
    end
    
end

grid on
xlabel('x')
ylabel('y')
zlabel('z')
if PointCloud == false
    axisLim = 1;
    title('Error Vector Plot - (Actual - Predicted)');    
    legend('Error Vector Length');
else
    axisLim = numSeg*SegLen;    
    title('Error Point Cloud Plot');
    legend('Pred','Act');
end
axis([-axisLim axisLim -axisLim axisLim -axisLim axisLim])

% Plot Histogram of error vector length
figure
hold off
histogram(eLen,'BinLimits',[1e-9,max(eLen)]);
title('Error Vector Length Histogram');


%% NN Test - Plot worst case error

errorLim = 0.2;
idx = eLen > errorLim;
% get dIn for worst performers
dIn_tmp = dIn(idx,:);
dOut_tmp = dOut(idx,:);

[len,xx] = size(dIn_tmp);

hold off

for i=1:len

    % Calc Position Predicted
    [Y,Xf,Af] = CalcBubbleVals(dIn_tmp(i,:)');
    
    disp('Predicted')
    disp(Y')

    dCouter_0 = Y(1);
    dCouter_120 = Y(2);
    dCouter_240 = Y(3);

    % Calculate the bend radius of the circle and angle theta of a single
    % arc formed when the outter bubbles expand (dC) for each bubble set
    % at 0,120,240 deg from the x axis

    [theta0,R0] = Calc_R_Theta(Cinner,dCouter_0,dR);
    [theta120,R120] = Calc_R_Theta(Cinner,dCouter_120,dR);
    [theta240,R240] = Calc_R_Theta(Cinner,dCouter_240,dR);

    % Calculate the bend angle (angle of rotation of the counter clockwise
    % toward the y axis)
    [BendAngle0] = CalcBendAngle(theta0,R0);
    [BendAngle120] = CalcBendAngle(theta120,R120);
    [BendAngle240] = CalcBendAngle(theta240,R240);

    [vNN] = CalcPosition(BendAngle0,BendAngle120,BendAngle240,SegLen,numSeg);


    % Calc Position Desired (dIn)
    dCouter_0 = dOut_tmp(i,1);
    dCouter_120 = dOut_tmp(i,2);
    dCouter_240 = dOut_tmp(i,3);

    disp('Actual')
    disp(dOut_tmp(i,:))

    % Calculate the bend radius of the circle and angle theta of a single
    % arc formed when the outter bubbles expand (dC) for each bubble set
    % at 0,120,240 deg from the x axis

    [theta0,R0] = Calc_R_Theta(Cinner,dCouter_0,dR);
    [theta120,R120] = Calc_R_Theta(Cinner,dCouter_120,dR);
    [theta240,R240] = Calc_R_Theta(Cinner,dCouter_240,dR);

    % Calculate the bend angle (angle of rotation of the counter clockwise
    % toward the y axis)
    [BendAngle0] = CalcBendAngle(theta0,R0);
    [BendAngle120] = CalcBendAngle(theta120,R120);
    [BendAngle240] = CalcBendAngle(theta240,R240);

    [v] = CalcPosition(BendAngle0,BendAngle120,BendAngle240,SegLen,numSeg);
    
    
    % Plot results
    %figure
    plot3(v(:,1),v(:,2),v(:,3),'-r');
    
    if i==1
        hold on
    end
    
    plot3(vNN(:,1),vNN(:,2),vNN(:,3),'-b');

    disp('Predicted Endpoint');
    disp(vNN(end,:));

    disp('Actual Endpoint');
    disp(v(end,:));
    
    
end

xlabel('x')
ylabel('y')
zlabel('z')
axisLim = numSeg*SegLen;
axis([-axisLim axisLim -axisLim axisLim 0 axisLim])
title('Worst Performance');
legend('Act','Pred');

grid on

%% FUNCTIONS 

% CalcPosition()
% This function take the change in outer diameter of the bubbles,
% calculates the bend angles for all three bubbles and then creates
% a plottable vector array in 3d space which represents the soft robot
% position and trajectory
function [v] = CalcPosition(BendAngle0,BendAngle120,BendAngle240,SegLen,numSeg)

    % Starting vector at origin with seg lenth in z direction
    p_origin = [0,0,SegLen];

    % Point Rotate quats
    % NOTE: Angle are negated to represent the reverse push of the bubbles
    %                     x     y      z
    B0_quat = quaternion([0 -BendAngle0 0],'euler','XYZ','point');
    B120_quat = quaternion([0 -BendAngle120 0],'euler','XYZ','point');
    B240_quat = quaternion([0 -BendAngle240 0],'euler','XYZ','point');

    % Frame Rotate quats around z axis
    rot = deg2rad(0);   
    r0_quat = quaternion([0 0 rot],'euler','XYZ','frame');
    rot = deg2rad(120); % pos rotates clockwise if looking down on z axis  
    r120_quat = quaternion([0 0 rot],'euler','XYZ','frame');
    rot = deg2rad(240);   
    r240_quat = quaternion([0 0 rot],'euler','XYZ','frame');


    % preallocate
    p = zeros(numSeg,3);
    p(1,:) = p_origin;

    % For each segment, calculate its combined rotation from each
    % of the bubbles (bubbles on 0, 120 and 240 degree angles)
    for i=2:numSeg

        p(i,:) = rotatepoint(B0_quat,p(i-1,:));

        % shift frame to 120 deg
        p(i,:) = rotateframe(r120_quat,p(i,:));

        % rotate 
        p(i,:) = rotatepoint(B120_quat,p(i,:));

        % shift frame to 240 deg
        p(i,:) = rotateframe(r120_quat,p(i,:));

        % rotate 
        p(i,:) = rotatepoint(B240_quat,p(i,:));

    % shift frame back to 0 deg
        p(i,:) = rotateframe(r120_quat,p(i,:));

    end

    % Add p (point) values to attach to previous segment each point to 
    % start at the end of the previous point thus linking all rotated
    % points
    for i=2:numSeg
        p(i,:) = p(i,:) + p(i-1,:);
    end

    % Create the line segment starting points (starting with the origin)
    o = zeros(numSeg,3);
    o(1,:) = [0,0,0]; % origin
    % Create origins
    for i=2:numSeg
        o(i,:) = p(i-1,:);
    end

    % Combine origin/points in alternating fashion to form a plotting vector
    v = zeros(numSeg*2,3);
    j=1;
    for i=1:numSeg
        v(j,:) = o(i,:);
        j= j+1;
        v(j,:) = p(i,:);
        j= j+1;
    end

end


% Calc_R_Theta()
% function to calculate theta and radius - theta is the angle from the 
% center of a circle to the end of the soft robot segment, r is the radius 
% of said circle, dC is the expansion of the outer balloon, dR is the robot
% witdth
function [theta,R] = Calc_R_Theta(Cinner,dC,dR)
    theta = dC/dR; % radians
    R = Cinner/theta;
end

% CalcBendAngle()
% This function calculate the bend angle from normal toward in inner
% circumfrence.  It is used to calculate how each segment moves in 
% comparison to the previous segment.
function [bendAngle] = CalcBendAngle(theta,R)
    c = pi*R^2; % circumfrence of the circle
    % Theta defines the arc angle of 1 segment
    % Bend angle is pi/2 - (pi-theta)/2
    bendAngle = pi/2 - (pi-theta)/2; % Bend angle in radians
end

% 25 hidden, MSE 9.30e-4, R .836, Scaled Conj training
function [Y,Xf,Af] = CalcBubbleValsXXX(X,~,~)
%MYNEURALNETWORKFUNCTION neural network simulation function.
%
% Auto-generated by MATLAB, 16-Feb-2022 09:10:49.
%
% [Y] = myNeuralNetworkFunction(X,~,~) takes these arguments:
%
%   X = 1xTS cell, 1 inputs over TS timesteps
%   Each X{1,ts} = Qx3 matrix, input #1 at timestep ts.
%
% and returns:
%   Y = 1xTS cell of 1 outputs over TS timesteps.
%   Each Y{1,ts} = Qx3 matrix, output #1 at timestep ts.
%
% where Q is number of samples (or series) and TS is the number of timesteps.

%#ok<*RPMT0>

% ===== NEURAL NETWORK CONSTANTS =====

% Input 1
x1_step1.xoffset = [-6.86998521223571;-6.86989332724689;1.47974239149883];
x1_step1.gain = [0.145560720890485;0.145562667768635;0.234734686660704];
x1_step1.ymin = -1;

% Layer 1
b1 = [-4.2104967357596416733;3.3153995889238214723;-4.2212406112776834632;-3.3716426501452603226;2.6182663584876828544;-2.2074054976613455281;-1.7934963389894100771;2.7851376949909565361;-1.6495021089502948097;-0.55955172401560815398;-1.5875586677687942849;-0.21958851387529276789;-0.15748481675989842166;0.030837297556887015293;0.44986562281374409267;1.157720006993262718;-1.999299386122565636;-1.3355945243521205601;-1.5875841170523803392;-2.3882471462351406721;-2.3121593315659483281;-3.0522587703565426942;-3.6979612453147434614;-4.1798870717710627787;4.423874230657082407];
IW1_1 = [2.2797451223621556515 -0.33290639206904831804 -2.2099975274445133877;-0.57947635660679497338 -2.5230566251597328353 3.0175433096607693351;-0.71485726498097246218 -1.3655037947953800881 -2.7601721133203445113;2.2566148895602053948 2.722095374978769744 -0.25353223419096315716;-3.378474581899556739 -0.10063956996523261422 -0.9292574912598638992;0.6647235625921691371 -3.2191131891890609751 1.991596290363531363;2.1090956156039184499 3.1890103292876448293 -1.0143698308376949413;-1.8336470487397953377 1.8443887305888295725 -1.3296466384358480717;1.9591120896200180379 -2.952315047258271985 1.1183512237259431998;2.3186514405270473027 -2.389308726730546617 2.1995525783946243692;-0.22562477174564454785 2.9364903133904394217 -0.012358963354930434589;-0.51932632856637883645 -3.0432261178662765033 2.1671810276028047326;-0.37467838943119208261 -0.23976697721779491324 3.0462389789206216761;-1.4302473599054745801 -2.1594305260105155497 0.37199172069829772314;1.831570190093851469 1.5132180162806008195 -2.0240998737876378932;1.558496338743766696 -2.1002534826436924931 2.5521339636751898539;-0.63719162599303824646 3.2300729067916100767 1.0770411031797342272;-3.7899060942919904349 0.20468871547025724311 -0.42935000473027973289;-0.54988685116999025393 1.4075368506918906064 2.5061641716710356675;-2.1972650520257981377 -1.2138252668079849705 -1.7192089516537096827;-2.1687791964133267264 -1.0934864415424565554 2.9013465390105670494;-1.9183078715138555115 1.962288307589271863 -1.6233772306670903873;-3.7550747226818494973 -0.035456874448925104759 0.1137992044851062895;-0.82946688571848226079 1.0581627332510550321 3.3176762458666626543;3.7476675440321414179 0.48933347183321157514 0.87171471726429461402];

% Layer 2
b2 = [-0.96031779994813804002;-0.20265441377775886278;0.14558169178768481933];
LW2_1 = [-0.42562770457789578238 0.054655208289279909317 0.10708468733360321234 -0.81650041137163376082 -0.45083903885515941301 0.2735595378505725761 -0.11620883921499126912 1.0501576615000089987 -0.16302956769152524008 -0.18001409693804043544 0.40311115020379761908 0.087271964231468934781 0.06870506990090276267 0.12086394838483185965 -0.21346324739206540277 -0.14682272318538622247 -0.01999758526720210941 0.036718759735708587921 -0.14853429443175017455 0.24302048852240537569 0.032446896065308789003 0.23396246790387359438 -0.27757858328410450754 0.22350841255738934654 -0.103413573506254311;0.53196692323129612934 -0.026387723027727901653 0.61819922160785778509 -0.56048825522358203255 -0.03398181743432006352 0.30174585890034011371 -0.12588150187028135374 0.41187019966290161488 0.10729694215564158921 -0.014492310291254955484 -0.088525918328267588775 -0.041074208531853478865 -0.15189285630703561192 -0.18946957365510538884 0.063536645178384712485 0.19475124946600694265 -0.18608436203164235856 0.094446501653241823893 -0.21846623087715863476 0.1335486191888383134 -0.057734912824044200275 -0.61174649906145861511 -0.63987990715406584119 0.74695398380622912704 0.094952669018920077271;0.2941345617710450222 -0.061476175929751214477 -0.59607447190378115565 -0.035039924061799371247 -0.44947974261979783961 0.20772187232109681765 0.087314233493339749037 -0.10665153063738409822 -0.086713722730881306888 -0.13186447323878938698 -0.027552211611572709204 -0.060306519813088869941 -0.037809388514091879507 -0.4841189183388425743 -0.12211876861885709311 -0.17468197632215287651 0.028216594255058322777 -0.064757845137660460799 0.10203010732514894432 -0.00033306858185571931686 0.16781481727701813567 0.17935586666082975471 -0.1438104826251152546 0.19255434629230888666 0.58874714287335727647];

% Output 1
y1_step1.ymin = -1;
y1_step1.gain = [33.3333333333333;33.3333333333333;33.3333333333333];
y1_step1.xoffset = [-0.03;-0.03;-0.03];

% ===== SIMULATION ========

% Format Input Arguments
isCellX = iscell(X);
if ~isCellX
    X = {X};
end

% Dimensions
TS = size(X,2); % timesteps
if ~isempty(X)
    Q = size(X{1},1); % samples/series
else
    Q = 0;
end

% Allocate Outputs
Y = cell(1,TS);

% Time loop
for ts=1:TS
    
    % Input 1
    X{1,ts} = X{1,ts}';
    Xp1 = mapminmax_apply(X{1,ts},x1_step1);
    
    % Layer 1
    a1 = tansig_apply(repmat(b1,1,Q) + IW1_1*Xp1);
    
    % Layer 2
    a2 = repmat(b2,1,Q) + LW2_1*a1;
    
    % Output 1
    Y{1,ts} = mapminmax_reverse(a2,y1_step1);
    Y{1,ts} = Y{1,ts}';
end

% Final Delay States
Xf = cell(1,0);
Af = cell(2,0);

% Format Output Arguments
if ~isCellX
    Y = cell2mat(Y);
end
end

% ===== MODULE FUNCTIONS ========

% Map Minimum and Maximum Input Processing Function
function y = mapminmax_apply(x,settings)
y = bsxfun(@minus,x,settings.xoffset);
y = bsxfun(@times,y,settings.gain);
y = bsxfun(@plus,y,settings.ymin);
end

% Sigmoid Symmetric Transfer Function
function a = tansig_apply(n,~)
a = 2 ./ (1 + exp(-2*n)) - 1;
end

% Map Minimum and Maximum Output Reverse-Processing Function
function x = mapminmax_reverse(y,settings)
x = bsxfun(@minus,y,settings.ymin);
x = bsxfun(@rdivide,x,settings.gain);
x = bsxfun(@plus,x,settings.xoffset);
end
