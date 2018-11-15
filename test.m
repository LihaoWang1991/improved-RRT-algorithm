%% pathRRT
%%  - create a path from a start node to an end node
%%    using the RRT algorithm.
%%  - RRT = Rapidly-exploring Random Tree
%%  
%% Based on code written by Yanjiang Zhao, improved by Lihao WANG

function pathRRT;

% create random world
Size = 100;
NumObstacles = 60;
speedSize=2;
world = createWorld(NumObstacles,[Size; Size],[0;0],speedSize);

% standard length of path segments
segmentLength = 5;

% randomly select start and end nodes
%start_node = generateRandomNode(world);
%end_node   = generateRandomNode(world);

start_node = [0,0,0,0,0];
end_node   = [80,80,0,0,0];

% establish tree starting with the start node
tree = start_node;
nodeDepth=[0];

% check to see if start_node connects directly to end_node
if ( (norm(start_node(1:2)-end_node(1:2))<segmentLength )...
    &(collision(start_node,end_node,world)==0) )
  path = [start_node; end_node];
else
  numPaths = 0;
  while numPaths<1,
      [tree,flag,nodeDepth] = extendTree(tree,end_node,segmentLength,world,nodeDepth);
      numPaths = numPaths + flag;
  end
end

% find path with minimum cost to end_node
path = findMinimumPath(tree,end_node);
pathDepth=nodeDepth(end);
plotWorld(world,path,tree,pathDepth);
tree
nodeDepth


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% createWorld
%%  - create random world with obstacles
%%  the first element is the north coordinate
%%  the second element is the south coordinate
function world = createWorld(NumObstacles, NEcorner, SWcorner, speedSize);

  % check to make sure that the region is nonempty
  if (NEcorner(1) <= SWcorner(1)) | (NEcorner(2) <= SWcorner(2)),
      disp('Not valid corner specifications!')
      world=[];
      
  % create world data structure
  else
    world.NumObstacles = NumObstacles;
    world.NEcorner = NEcorner;
    world.SWcorner = SWcorner;
                          
    % create NumObstacles 
    %maxRadius = min(NEcorner(1)- SWcorner(1), NEcorner(2)-SWcorner(2));
    %maxRadius = 5*maxRadius/NumObstacles/2;
    maxRadius = 3;
    for i=1:NumObstacles,
        % randomly pick radius
        world.radius(i) = maxRadius*rand;
        % randomly pick center of obstacles
        cn = SWcorner(1) + world.radius(i)...
            + (NEcorner(1)-SWcorner(1)-2*world.radius(i))*rand;
        ce = SWcorner(2) + world.radius(i)...
            + (NEcorner(2)-SWcorner(2)-2*world.radius(i))*rand;
        world.cn(i) = cn;
        world.ce(i) = ce;
        speedChoice=[1,0;0.707,0.707;0,1;-0.707,0.707;-1,0;-0.707,-0.707;0,-1;0.707,-0.707];
        if rem(i,2)==0 & rem(i/2,9)~=0,
            world.speed(i,:)=speedChoice(rem(i/2,9),:)*speedSize;
        else 
            world.speed(i,:)=[0,0];
        end
    end
    world.speed
  end
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% obstaclesAtStepK
%% calculate the obstacles' positions at step k
function [obstaclesK]=obstaclesAtStepK(world,k);  

obstaclesK=[(world.cn)',(world.ce)']+world.speed*k;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% generateRandomNode
%%   create a random node (initialize)
function node=generateRandomNode(world);

% randomly pick configuration
pn       = (world.NEcorner(1)-world.SWcorner(1))*rand;
pe       = (world.NEcorner(2)-world.SWcorner(2))*rand;
chi      = 0;
cost     = 0;
node     = [pn, pe, chi, cost, 0];

% check collision with obstacle
while collision(node, node, world),
  pn       = (world.NEcorner(1)-world.SWcorner(1))*rand;
  pe       = (world.NEcorner(2)-world.SWcorner(2))*rand;
  chi      = 0;
  cost     = 0;
  node     = [pn, pe, chi, cost, 0];
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% collision
%%   check to see if a node is in collsion with obstacles
function collision_flag = collision(node, parent, world);

collision_flag = 0;

if ((node(1)>world.NEcorner(1))...
    | (node(1)<world.SWcorner(1))...
    | (node(2)>world.NEcorner(2))...
    | (node(2)<world.SWcorner(2)))
  collision_flag = 1;
else
    for sigma = 0:.2:1,
    p = sigma*node(1:2) + (1-sigma)*parent(1:2);
      % check each obstacle
      for i=1:world.NumObstacles,
        if (norm([p(1);p(2)]-[world.cn(i); world.ce(i)])<=1.5*world.radius(i)),
            collision_flag = 1;
            break;
        end
      end
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% dynaCollision
%%   check to see if a node is in collsion with one certain moving obstacle
function dynaCollision_flag = dynaCollision (node, parent, world, k, i);  % k is the step number and i is the moving obstacle index
dynaCollision_flag = 0;
Last_Obstacle=obstaclesAtStepK(world,k-1);
Current_Obstacle=obstaclesAtStepK(world,k);
Next_Obstacle=obstaclesAtStepK(world,k+1);
%% calculate the obstacle path start and end point coordinates at step k and k+1 taken obstacle radius into account
O10=Last_Obstacle(i,1:2)-world.speed(i)/2*world.radius(i);
O20=Current_Obstacle(i,1:2)+world.speed(i)/2*world.radius(i);
O30=Current_Obstacle(i,1:2)-world.speed(i)/2*world.radius(i);
O40=Next_Obstacle(i,1:2)+world.speed(i)/2*world.radius(i);
%% calculate the 4 vertex coordinates of the sweeping rectangle at step k and k+1
SpeedNormal=[world.speed(i,2),(-1)*world.speed(i,1)];
O11=O10+SpeedNormal/2*world.radius(i);
O21=O20+SpeedNormal/2*world.radius(i);
O31=O30+SpeedNormal/2*world.radius(i);
O41=O40+SpeedNormal/2*world.radius(i);
O12=O10-SpeedNormal/2*world.radius(i);
O22=O20-SpeedNormal/2*world.radius(i);
O32=O30-SpeedNormal/2*world.radius(i);
O42=O40-SpeedNormal/2*world.radius(i);
StartPoint1=[O11;O12;O22;O21];
EndPoint1=[O12;O22;O21;O11];
StartPoint2=[O31;O32;O42;O41];
EndPoint2=[O32;O42;O41;O31];
%% check the collision between the parent to node path and the sweeping rectangle at step k
for j=1:4,
    if ((max(parent(1),node(1))>min(StartPoint1(j,1),EndPoint1(j,1)))...
        &(min(parent(1),node(1))<max(StartPoint1(j,1),EndPoint1(j,1)))),    % in case that the parent to node path and each rectangle edge the has commun part in x axis
        if world.speed(i,1)==0,                                             % in case of rectangle edge O12-O22 and O21-O11 vertical to x axis
            if (j==2 | j==4),                                               
                Common=StartPoint1(j,1);
                yPath=returnY(parent, node, Common);
                y1Edge=StartPoint1(j,2);
                y2Edge=EndPoint1(j,2);
                if (yPath-y1Edge)*(yPath-y2Edge)<=0;
                    dynaCollision_flag=1;
                    return
                end
            end
            
        elseif world.speed(i,2)==0,                                         % in case of rectangle edge O11-O12 and O21-O22 vertical to x axis
            if (j==1 | j==3),                                               
                Common=StartPoint1(j,1);
                yPath=returnY(parent, node, Common);
                y1Edge=StartPoint1(j,2);
                y2Edge=EndPoint1(j,2);
                if (yPath-y1Edge)*(yPath-y2Edge)<=0;
                    dynaCollision_flag=1;
                    return
                end
            end
            
        else
            CommonMin=min([parent(1), node(1), StartPoint1(j,1), EndPoint1(j,1)]);  % lower bound of the x common part
            CommonMax=max([parent(1), node(1), StartPoint1(j,1), EndPoint1(j,1)]);  % upper bound of the x common part
            y1Path=returnY(parent, node, CommonMin);
            y2Path=returnY(parent, node, CommonMax);
            y1Edge=returnY(StartPoint1(j,:),EndPoint1(j,:), CommonMin);
            y2Edge=returnY(StartPoint1(j,:),EndPoint1(j,:), CommonMax);
            if (y1Path-y1Edge)*(y2Path-y2Edge)<=0;
            dynaCollision_flag=1;
            return
            end
        end
    end
end      
    
%% check the collision between the node point and the sweeping rectangle at step k+1
cosTheta=world.speed(i,1)/norm(world.speed(i,:));                                 % rotate the coordinate system by angle Theta, Theta is the angle of the speed vector
sinTheta=world.speed(i,2)/norm(world.speed(i,:));
NewO31(1)=O31(1)*cosTheta+O31(2)*sinTheta;                                        % calculate the new coordinate of each points in new coordinate system
NewO31(2)=O31(2)*cosTheta-O31(2)*sinTheta;
NewO41(1)=O41(1)*cosTheta+O41(2)*sinTheta;
NewO41(2)=O41(2)*cosTheta-O41(2)*sinTheta;
NewO32(1)=O32(1)*cosTheta+O32(2)*sinTheta;
NewO32(2)=O32(2)*cosTheta-O32(2)*sinTheta;
NewO42(1)=O42(1)*cosTheta+O42(2)*sinTheta;
NewO42(2)=O42(2)*cosTheta-O42(2)*sinTheta;
NewNode(1)=node(1)*cosTheta+node(2)*sinTheta;
NewNode(2)=node(2)*cosTheta-node(2)*sinTheta;
if (NewNode(1)>NewO31(1) & NewNode(1)<NewO41(1) & NewNode(2)>NewO31(2) & NewNode(2)<NewO32(2) )  % check if NewNode falls into obstacle sweeping rectangle at k+1
    dynaCollision_flag=1;
end
    
    

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% returnY
%% given 2 endpoints a(x1,y1) and b(x2,y2) of a segment, return the y value at another point x on the segment
function y = returnY (a, b, x);
if a(1)==b(1),   % check if the segment is in parallel with y axis
    disp('Not valid segment')
else
    y=a(2)+(b(2)-a(2))/(b(1)-a(1))*(x-a(1));
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% canEndConnectToTree
%%   check to see if the end node can connect to the tree
function flag = canEndConnectToTree(tree,end_node,minDist,world);
  flag = 0;
  % check only last node added to tree since others have been checked
  if ( (norm(tree(end,1:2)-end_node(1:2))<minDist)...
     & (collision(tree(end,1:2), end_node(1:2), world)==0) ),
    flag = 1;
  end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% extendTree
%%   extend tree by randomly selecting point and growing tree toward that
%%   point
function [new_tree,flag,new_nodeDepth] = extendTree(tree,end_node,segmentLength,world,nodeDepth); %nodeDepth is the step number for each node for dynamic collicision check

  flag1 = 1;
  while flag1==1,
    flag1=0;   % initialize flag1 to 1, flag1==0 means that collision check is passed, flag1==1 means that collision exists
    % select a random point
    randomPoint = [...
        (world.NEcorner(1)-world.SWcorner(1))*rand,...
        (world.NEcorner(2)-world.SWcorner(2))*rand];
    % find leaf on node that is closest to randomPoint
    tmp = tree(:,1:2)-ones(size(tree,1),1)*randomPoint; % times ones is to make sure the same dimention, because tree maybe has n rows
    [dist,idx] = min(sum(tmp.^2,2)); %diag在这里是取矩阵对角线元素输出列向量，tmp和其转置相乘对角线正好是两坐标相减的平方和，即为距离的平方，看离哪个点最近并指示出其位置idx
    cost     = tree(idx,4) + segmentLength;%离哪个已知点近就加在哪个点上...
    new_point = (randomPoint-tree(idx,1:2));
    new_point = tree(idx,1:2)+new_point/norm(new_point)*segmentLength;
    new_node = [new_point, 0, cost, idx];%离哪个已知点近就加在哪个点上，并记录路线idx
    % collision check for both static and moving obstacles
    obstaclesLastStep=obstaclesAtStepK(world,nodeDepth(idx));
    obstaclesNowStep=obstaclesAtStepK(world,nodeDepth(idx)+1);
    i=1;
    while (i<=world.NumObstacles) & (flag1==0),
        if ( norm (obstaclesLastStep(i,:)-tree(idx,1:2))<10 | norm (obstaclesNowStep(i,:)-tree(idx,1:2))<10 ),   %%analyze only the obstacles at a distance less than 10 from the parent node
            if rem(i,2)==0 & rem(i/2,9)~=0,  % collision check for moving obstacles
                flag1 = dynaCollision (new_point, tree(idx,1:2), world, nodeDepth(idx), i);
                if flag1==1,
                    break;
                end
            else   % collision check for static obstacles
                for sigma = 0:.1:1,                
                    p = sigma*new_point + (1-sigma)*tree(idx,1:2);
                       if (norm([p(1);p(2)]-[world.cn(i); world.ce(i)])<=1.5*world.radius(i)),
                           flag1=1; 
                           break;
                       end
                end
            end 
        end
    i=i+1;    
    end
    
    
    if flag1==0,   % if collision check is passed, add the new_node to the tree
        new_tree = [tree; new_node];
        new_nodeDepth=[nodeDepth; nodeDepth(idx)+1];
    end
  end
 
  % check to see if new node connects directly to end_node
  if ( (norm(new_node(1:2)-end_node(1:2))<segmentLength )...
      &(collision(new_node,end_node,world)==0) )
    flag = 1;
    new_tree(end,3)=1;  % mark node as connecting to end.
  else
    flag = 0;
  end
  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% findMinimumPath
%%   find the lowest cost path to the end node
function path = findMinimumPath(tree,end_node);
    
    % find nodes that connect to end_node
    connectingNodes = [];
    for i=1:size(tree,1),
        if tree(i,3)==1,
            connectingNodes = [connectingNodes; tree(i,:)];
        end
    end

    % find minimum cost last node
    [tmp,idx] = min(connectingNodes(:,4));
    
    % construct lowest cost path
    path = [connectingNodes(idx,:); end_node];
    parent_node = connectingNodes(idx,5);
    while parent_node>1,
        path = [tree(parent_node,:); path];
        parent_node = tree(parent_node,5); 
    end
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% plotWorld
%%   plot obstacles and path
function plotWorld(world,path,tree,pathDepth)
  % the first element is the north coordinate
  % the second element is the south coordinate

  N = 10;
  th = 0:2*pi/N:2*pi;
  figure(1), clf
  hold on
  axis equal;
  axis([-5,105,-5,105]);
  X=[];
  Y=[];
  Xpath = path(:,1);
  Ypath = path(:,2);
  p1=plot(X,Y,'k');
  P2=plot(Xpath(1),Ypath(1),'r','linewidth',3);
  

  
  pObstacles=[];
  for i=1:world.NumObstacles,
      pObstacles(i)=plot(world.cn(i),world.ce(i),'color','b','marker','.','markersize',20*world.radius(i));
  end
  
  pPath=plot(Xpath(1),Ypath(1),'r','linewidth',1);
  
  for j=0:pathDepth
      
    for i=1:world.NumObstacles,
         %X = world.radius(i)*cos(th) + world.cn(i)+j*world.speed(i,1);
         %Y = world.radius(i)*sin(th) + world.ce(i)+j*world.speed(i,2);
         %plot(X,Y,'k');
         X=world.cn(i)+j*world.speed(i,1);
         Y=world.ce(i)+j*world.speed(i,2);
         set(pObstacles(i),'xdata',X,'ydata',Y);
    end
    
    Xpath = path(1:j+1,1);
    Ypath = path(1:j+1,2);
    set(pPath,'xdata',Xpath,'ydata',Ypath);

    %set(p1,'xdata',X,'ydata',Y);
    %set(p2,'xdata',[Xpath(j+1),Xpath(j+2)],'ydata',[Ypath(j+1),Ypath(j+2)]);
    %plot([Xpath(j+1),Xpath(j+2)],[Ypath(j+1),Ypath(j+2)],'r','linewidth',3);
    pause(0.3);
    drawnow

  end
  
   
  for i=2:size(tree,1);
      plot([tree(tree(i,5),1), tree(i,1)], [tree(tree(i,5),2), tree(i,2)])
  end
  

  
  
