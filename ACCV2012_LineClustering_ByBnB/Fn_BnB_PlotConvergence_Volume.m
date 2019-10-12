function Fn_BnB_PlotConvergence_Volume(global_volume)


     figure('Name','convergence of volume');
%plot(global_volume,'.r-'),
% plot(global_volume,'.-r'),
semilogy(global_volume,'.-r'),

title('Convergence of volume'), xlabel('Iterations'),ylabel('remaining volume (in%)'),...
   grid on;