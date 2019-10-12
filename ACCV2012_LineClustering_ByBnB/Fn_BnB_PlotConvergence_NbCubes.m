function Fn_BnB_PlotConvergence_NbCubes(ListNbCubes)


     figure('Name','evolution of the nb of cubes');
% plot(ListNbCubes,'.-r');
semilogy(ListNbCubes','.-r')

title('evolution of the nb of cubes'), xlabel('Iterations'),ylabel('Number of cubes'),...
   grid on;