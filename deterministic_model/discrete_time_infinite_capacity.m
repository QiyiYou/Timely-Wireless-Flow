clear all; close all;
% number of flows
K = 3;
% the delay of all flows
D_max = 6*ones(1,K);

isFeasible = zeros(D_max(1), D_max(2), D_max(3));

for D1=1:D_max(1)
    for D2=1:D_max(2)
        for D3=1:D_max(3)
            D = [D1, D2, D3]
            [isFeasible(D1,D2,D3), x_opt] = isFeasibleInfCapacity(K,D);
        end
    end
end

for D1=1:D_max(1)
    figure;
    set(gca,'FontSize',20);
    hold on;
    for D2=1:D_max(2)
        for D3=1:D_max(3)
            if(isFeasible(D1,D2,D3) == 1)
                plot(D2, D3, 'bo', 'Linewidth', 6);
            else
                plot(D2, D3, 'ro', 'Linewidth', 6);
            end
        end
    end
    hold off;
    xlim([1,D_max(2)]);
    ylim([1,D_max(2)]);
    xlabel('$D_2$','FontSize', 20, 'FontName', 'Arial', 'Interpreter', 'latex');
    ylabel('$D_3$','FontSize', 20, 'FontName', 'Arial', 'Interpreter', 'latex');
    title(sprintf('$D_1=%d$',D1),'FontSize', 20, 'FontName', 'Arial', 'Interpreter', 'latex');
    box on;
    grid on;
    export_fig(sprintf('fig/D1=%d', D1), '-pdf','-transparent','-nocrop');
end


