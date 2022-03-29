function plot_error(T0c)
tdraw = 20;
S_length = 0.025*2*2*pi+0.05*sqrt(2);
ts_1 = round(t_draw/S_length*0.025*2*pi);
ts_2 = tdraw-2*ts_1;
H_length = 3*0.1+2*0.05;
th_1 = round(20/H_length*0.1);
th_2 = (20-3*th_1)/2;
B_length = 0.1+0.075*4+0.025*2*pi;
tb_1 = round(20/B_length*0.1);
tb_2 = round(20/B_length*0.075);
tb_3 = (tdraw-tb_1-4*tb_2)/2;
%% tb
for t = 0:0.01:20
    if 
