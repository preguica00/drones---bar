function [m_drone,m_bar,m_sys,g, C_barra] = parameters
   


%masses
m_drone= 1.12;
m_bar=0.5;
m_sys = 2*m_drone+ m_bar;
g=10;


C_barra = 0.2;



end

