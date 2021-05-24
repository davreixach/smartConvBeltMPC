% clear;

BEGIN_ACADO;                                    % Always start with "BEGIN_ACADO". 
    
    acadoSet('problemname', 'robot_sp');        % Set your problemname. If you 
                                                % skip this, all files will
                                                % be named "myAcadoProblem"
                                                   
    acadoSet('results_to_file', false);         % Write results only to workspace
                                                % not to file.
    
    
    DifferentialState x y theta;                % Differential States:
                                                % x,y,angle
                                            
    Control v omega;                            % Control: 
                                                % Linear & Angular Velocity
                                            
      

%     B=100;                                    % Static parameters

    
    %% Differential Equation
    f = acado.DifferentialEquation();       % Set the differential equation object
    
    f.add(dot(x) == v*sin(theta));          % Write down your ODE
    f.add(dot(y) == v*cos(theta));
    f.add(dot(theta) == omega);
    
    
    %% Optimal Control Problem
    ocp = acado.OCP(0.0, Hp*Ts, 20);           % Set up the Optimal Control Problem (OCP)
                                            % Start at 0s, control in 20
                                            % intervals upto 1s

    h={x,y,theta,v,omega};                               % the LSQ-Function

    QW = blkdiag(Q,R);                       % The weighting matrix
    
    r = REF;                         % The reference
        
    ocp.minimizeLSQ( QW, h, r );            % Minimize this Least Squares Term
    %ocp.minimizeLSQ( h, r );               % (Other possibility)
    %ocp.minimizeLSQ( h );                  % (Other possibility)
    
    ocp.subjectTo( f );                     % Your OCP is always subject to your 
                                            % differential equation

    ocp.subjectTo(  -bu(1) <= v <= bu(1) );       % Bounds
    ocp.subjectTo(  -bu(2) <= omega <= bu(2) );       % Bounds
    
%     ocp.subjectTo( 'AT_END', T == Tss );    % Terminal constraint
%     ocp.subjectTo( 'AT_END', Ca == Cass );    % Terminal constraint
%     ocp.subjectTo( 'AT_END', Tss-.01 <= T <= Tss+.01 )     
%     ocp.subjectTo( 'AT_END', Cass-.001 <= Ca <= Cass+.001 )

   
    
    %% SETTING UP THE (SIMULATED) PROCESS
    identity = acado.OutputFcn();
    dynamicSystem = acado.DynamicSystem(f, identity);    % Set up a dynamic system with
                                                         % the differential equation and an
                                                         % output function
                                                         
    process = acado.Process(dynamicSystem, 'INT_RK45');  % Simulates the process to be controlled 
                                                         % based on a dynamic model.
                                                         % The class Process is one of the two main 
                                                         % building-blocks within the SimulationEnvironment 
                                                         % and complements the Controller. It simulates the 
                                                         % process to be controlled based on a dynamic model.
 
    
    
    %% SETTING UP THE MPC CONTROLLER:
    algo = acado.RealTimeAlgorithm(ocp, Ts);          % The class RealTimeAlgorithm serves as a user-interface 
                                                        % to formulate and solve model predictive control problems.
                                                        
    %algo.set('MAX_NUM_ITERATIONS', 2 );                 % Set some algorithm parameters
%     algo.set('KKT_TOLERANCE', 1e-2);
%     algo.set('INTEGRATOR_TOLERANCE',1e-2);
%     algo.set('ABSOLUTE_TOLERANCE',1e-2);

    r = REF;
    
    Reference = acado.StaticReferenceTrajectory( r );  % Allows to define a static reference trajectory that 
                                                        % the ControlLaw aims to track. 
                                                        
    controller = acado.Controller( algo,Reference ); % The controller complements a Process. 
                                                         % It contains an online control law for
                                                         % obtaining the control inputs of a process
    
    
    %% SETTING UP THE SIMULATION ENVIRONMENT,  RUN THE EXAMPLE..
    sim = acado.SimulationEnvironment( 0.0,Hs*Ts,process,controller ); % Setup the closed-loop simulations of dynamic systems. 
                                                                    % Simulate from 0 to 3 sec
    
    r = ref(:,1)';                                                   % Initilize the states
    sim.init( r );
    

    
END_ACADO;           % Always end with "END_ACADO".
                     % This will generate a file problemname_ACADO.m. 
                     % Run this file to get your results. You can
                     % run the file problemname_ACADO.m as many
                     % times as you want without having to compile again.
              
                     
% Run the test
% out = tank_reactor_RUN();