function simulate()

    close all
    clf
    hold on
    tic
    
  [m_drone,m_bar,m_sys,g, C_barra] = parameters;
    [H,Ts,drone1_info, drone2_info, bar_info] = system_info;

%     plot_prediction_bar = plot(0,0,'om-', 'Linewidth', 1);
    plot_trajectory_bar = plot(C_barra/2,C_barra/2,'or-','Linewidth', 1);

%     plot_trajectory_d1 = plot(0,C_barra/2,'ob-','Linewidth', 1.5);
%     plot_trajectory_d2 = plot(C_barra,C_barra/2,'ok-','Linewidth', 1.5);
%     
    plot_barra15 = plot(0,0,'Color',[1 0.6 0],'LineWidth',3);
    plot_barra25 = plot(0,0,'Color',[1 0.6 0],'LineWidth',3);
    plot_barra35 = plot(0,0,'Color',[1 0.6 0],'LineWidth',3);
    plot_barra5 = plot(0,0,'Color',[1 0.6 0],'LineWidth',3);

    state_trajectory=[];
    control_variables=[];
    commands=[];
    axis square
%     ylim([-0.5 3.5])
%     xlim([-0.5 2.01])
%     
    xbarra15=[];
    ybarra15=[];
    yawbarra15=[];
    
    xbarra25=[];
    ybarra25=[];
    yawbarra25=[];  
    
    xbarra35=[];
    ybarra35=[];
    yawbarra35=[];   
    
    xbarra5=[];
    ybarra5=[];
    yawbarra5=[];

    
    current_state = [0*ones(12,1);C_barra/2;C_barra/2;zeros(10,1)];
    
    current_MPC_solution = [];


%        set(gca,'nextplot','replacechildren','visible','off')
%     f = getframe;
%     [im,map] = rgb2ind(f.cdata,306,'nodither');
%     im(1,1,1,40) = 0;


    for k = 1:50
        %% Run the controller

        [command, current_MPC_solution, predicted_trajectory] = ...
            optimizetrajectory(current_state, current_MPC_solution,k);
        
        %% Run the simulation
        
        current_state = simulate_timestep(current_state, command);

        %% Visualize
%         plot_prediction_bar.XData = predicted_trajectory(:,13);
%         plot_prediction_bar.YData = predicted_trajectory(:,14);

        plot_trajectory_bar.XData(end+1) = current_state(13);
        plot_trajectory_bar.YData(end+1) = current_state(14);
        
%         plot_trajectory_d1.XData(end+1) = current_state(13)- ( (C_barra/2)*cos(current_state(21)) );
%         plot_trajectory_d1.YData(end+1) = current_state(14)- ( (C_barra/2)*sin(current_state(21)) );
% 
%         
%         plot_trajectory_d2.XData(end+1) = current_state(13)+( (C_barra/2)*cos(current_state(21)) );
%         plot_trajectory_d2.YData(end+1) = current_state(14)+( (C_barra/2)*sin(current_state(21)) );

    
%         if k==15
%             xbarra15= current_state(13);
%             ybarra15= current_state(14);
%             yawbarra15=current_state(21);
%             
%             plot_barra15.XData =  [ C_barra*cos(yawbarra15) 0]+ xbarra15- ( (C_barra/2)*cos(yawbarra15) );
%             plot_barra15.YData =  [ 0 C_barra*sin(yawbarra15)]+ybarra15- ( (C_barra/2)*sin(yawbarra15) );
%             
%         end
%         
%         if k==25
%             xbarra25= current_state(13);
%             ybarra25= current_state(14);
%             yawbarra25=current_state(21);
%             
%             plot_barra25.XData =  [ C_barra*cos(yawbarra25) 0]+ xbarra25- ( (C_barra/2)*cos(yawbarra25) );
%             plot_barra25.YData =  [ 0 C_barra*sin(yawbarra25)]+ybarra25- ( (C_barra/2)*sin(yawbarra25) );
%             
%         end  
%         
%         if k==35
%             xbarra35= current_state(13);
%             ybarra35= current_state(14);
%             yawbarra35=current_state(21);
%             
%             plot_barra35.XData =  [ C_barra*cos(yawbarra35) 0]+ xbarra35- ( (C_barra/2)*cos(yawbarra35) );
%             plot_barra35.YData =  [ 0 C_barra*sin(yawbarra35)]+ybarra35- ( (C_barra/2)*sin(yawbarra35) );
%             
%         end
%         
%         if k==5
%             xbarra5= current_state(13);
%             ybarra5= current_state(14);
%             yawbarra5=current_state(21);
%             
%             plot_barra5.XData =  [ C_barra*cos(yawbarra5) 0]+ xbarra5- ( (C_barra/2)*cos(yawbarra5) );
%             plot_barra5.YData =  [ 0 C_barra*sin(yawbarra5)]+ybarra5- ( (C_barra/2)*sin(yawbarra5) );
%             
%         end
        
%         if k>10
        state_trajectory(end+1,:) = current_state;
        control_variables(end+1,:) = current_MPC_solution;
        commands(end+1,:) = command;
%         end

%     f = getframe;
%   im(:,:,1,k) = rgb2ind(f.cdata,map,'nodither');
%       

        drawnow
        pause(0.05)
        
%         legend([plot_trajectory_d1 plot_trajectory_d2, plot_trajectory_bar],'Drone 1','Drone 2', 'bar','Location','Northeast')

        aa=k

    end

% imwrite(im,map,'drone_trajetoria1.gif','DelayTime',0.1,'LoopCount',inf) %g443800
    
    %T1
    figure
    plot(commands(:,1));
    
    %taux1,tauy1,tauz1,
    figure
    plot(commands(:,2),'b');
    hold on;
    plot(commands(:,3),'r');
    hold on;
    plot(commands(:,4),'k');
legend({'\tau_{x1}', '\tau_{y1}','\tau_{z1}'}, 'Interpreter','latex','FontSize',12)

    %T2
    figure
    plot(commands(:,5));
        
    %taux2,tauy2,tauz2,
    figure
    plot(commands(:,6),'b');
    hold on;
    plot(commands(:,7),'r');
    hold on
    plot(commands(:,8),'k');
legend({'\tau_{x2}', '\tau_{y2}','\tau_{z2}'}, 'Interpreter','latex','FontSize',12)

save('states.mat','state_trajectory')
save('control.mat','control_variables')

toc
end
