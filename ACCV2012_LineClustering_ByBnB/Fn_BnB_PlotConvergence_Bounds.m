function hfig=Fn_BnB_PlotConvergence_Bounds(lowerbounds, upperbounds)

    hfig=figure('Name','convergence of lower and upper bounds');
    plot(upperbounds,'r.-')
    hold on; plot(lowerbounds,'b.-'); grid on; hold off;
    
    title('Convergence of bounds'), xlabel('Iterations'),ylabel('Upper and lower bounds'),...
    legend('upper', 'lower', 'Location', 'BestOutside')
