function current_state = simulate_timestep(current_state,command)
 
[H,Ts,drone1_info, drone2_info, bar_info] = system_info;

    tspan = [0 Ts];



    my_ode = @(t,y) system_ode(t,y,command);
    [~, y] = ode45(my_ode,tspan, current_state);
    
    current_state = [y(end,1), y(end,2),y(end,3),y(end,4),y(end,5),y(end,6),y(end,7),y(end,8),y(end,9),y(end,10),y(end,11),y(end,12),y(end,13),y(end,14),y(end,15),y(end,16),y(end,17),y(end,18),y(end,19),y(end,20),y(end,21),y(end,22),y(end,23),y(end,24)];
   
 
end