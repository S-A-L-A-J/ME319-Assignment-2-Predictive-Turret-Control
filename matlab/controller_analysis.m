function metrics = controller_analysis()
%CONTROLLER_ANALYSIS First-principles plant and frozen LTI-loop evidence.
root=fileparts(fileparts(mfilename('fullpath'))); out=fullfile(root,'figures');
if ~exist(out,'dir'), mkdir(out); end
s=tf('s'); Jt=1200; Jm=0.20; N=100; b=1500; Kt=10; Ke=10; R=1; La=0.10;
Jeq=Jm+Jt/N^2; beq=b/N^2;
Gp=(Kt/N)/(s*((La*s+R)*(Jeq*s+beq)+Kt*Ke));
Gc=7000/(0.05*s+1); L=Gc*Gp; T=feedback(L,1);
[gm,pm,~,wcp]=margin(L); info=stepinfo(T);
metrics=table(Jeq,beq,wcp,pm,20*log10(gm),info.SettlingTime,info.Overshoot,...
    'VariableNames',{'J_eq','b_eq','crossover_rad_s','phase_margin_deg','gain_margin_dB','settling_time_s','overshoot_percent'});
writetable(metrics,fullfile(root,'results','matlab_controller_metrics.csv')); disp(metrics);
w=logspace(-1,3,1200); [mag,phase]=bode(L,w); mag=squeeze(mag); phase=squeeze(phase);
f=figure('Visible','off','Color','w','Position',[100 100 900 620]);
subplot(2,1,1); semilogx(w,20*log10(mag),'b'); yline(0,'k--'); grid on; ylabel('Magnitude [dB]');
title(sprintf('Frozen loop: PM %.1f deg, GM %.1f dB, crossover %.2f rad/s',pm,20*log10(gm),wcp));
subplot(2,1,2); semilogx(w,phase,'b'); yline(-180,'k--'); grid on; ylabel('Phase [deg]'); xlabel('Frequency [rad/s]');
style_figure(f);
exportgraphics(f,fullfile(out,'matlab_loop_margins.png'),'Resolution',180); close(f);
f=figure('Visible','off','Color','w'); step(T,2); grid on; title('Frozen linear closed-loop step response'); style_figure(f);
exportgraphics(f,fullfile(out,'matlab_closed_loop_step.png'),'Resolution',180); close(f);
end

function style_figure(f)
set(f,'Color','w'); ax=findall(f,'Type','axes');
for k=1:numel(ax)
    set(ax(k),'Color','w','XColor','k','YColor','k','GridColor',[.7 .7 .7]);
    set(get(ax(k),'Title'),'Color','k'); set(get(ax(k),'XLabel'),'Color','k'); set(get(ax(k),'YLabel'),'Color','k');
end
lg=findall(f,'Type','legend'); set(lg,'Color','w','TextColor','k','EdgeColor','k');
tx=findall(f,'Type','text'); set(tx,'Color','k');
end
