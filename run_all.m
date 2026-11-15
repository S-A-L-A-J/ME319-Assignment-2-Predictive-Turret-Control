function summary = run_all(nMC)
%RUN_ALL Authoritative MATLAB simulation for ME319 standardized cases A/B/C.
% Uses the assignment plant/controller interfaces, one frozen design, 0-10 s.
if nargin < 1, nMC = 20; end
root = fileparts(mfilename('fullpath'));
addpath(fullfile(root,'matlab'),fullfile(root,'matlab','strategies'));
if ~exist(fullfile(root,'results'),'dir'), mkdir(fullfile(root,'results')); end
if ~exist(fullfile(root,'figures'),'dir'), mkdir(fullfile(root,'figures')); end

p.dt=0.05; p.T=10; p.tau=0.5; p.sig_r=1; p.sig_el=0.02*pi/180;
thetaIMM=[2 10]; H=round(p.tau/p.dt); N=round(p.T/p.dt)+1;
S=load(fullfile(root,'data','scenarios.mat')); data=S.data;
dt=0.0005; tsim=0:dt:p.T; ns=numel(tsim); tmeas=(0:N-1)*p.dt;

scenario={}; run_id=[]; est_rmse=[]; pred_rmse=[]; los_rmse_deg=[];
max_voltage=[]; pulse_count=[]; success=[]; first_hit_s=[]; best_miss_m=[];

for s=1:numel(data)
    d=data(s); nUse=min(nMC,size(d.r,1));
    xtrue=interp1(tmeas,d.x(1,1:N),tsim,'linear');
    ytrue=interp1(tmeas,d.x(2,1:N),tsim,'linear');
    losTrue=atan2(ytrue,xtrue);
    for m=1:nUse
        [xh,xp]=imm_cvca(d.r(m,:),d.el(m,:),d.valid,p,thetaIMM);
        kEval=(round(1/p.dt)+1):N;
        ee=sum((xh-d.x(:,1:N)).^2,1);
        ep=sum((xp-d.x(:,1+H:N+H)).^2,1);

        clear turret_controller
        theta=atan2(d.r(m,1)*sin(d.el(m,1)),d.r(m,1)*cos(d.el(m,1)));
        thetaDot=0; current=0; rHold=d.r(m,1); elHold=d.el(m,1);
        vLog=zeros(1,ns); thLog=zeros(1,ns); fireLog=false(1,ns);
        prevFire=false; lastAccepted=-inf; pulses=0; hit=false; firstHit=NaN; bestMiss=inf;
        for q=1:ns
            t=tsim(q);
            jm=round(t/p.dt)+1;
            isSample=(jm>=1 && jm<=N && abs(t-(jm-1)*p.dt)<=dt/4);
            if isSample, rHold=d.r(m,jm); elHold=d.el(m,jm); validNow=d.valid(jm); else, validNow=false; end
            [V,fire]=turret_controller(t,rHold,elHold,validNow,theta,thetaDot);
            V=max(min(V,400),-400);
            vLog(q)=V; thLog(q)=theta; fireLog(q)=logical(fire);
            if fire && ~prevFire && t-lastAccepted>=1-1e-9
                miss=ray_miss(hypot(xtrue(q),ytrue(q)),theta-losTrue(q));
                pulses=pulses+1; lastAccepted=t; bestMiss=min(bestMiss,miss);
                if miss<=2 && ~hit, hit=true; firstHit=t; end
            end
            prevFire=logical(fire);
            if q<ns
                [thetaRate,thetaDDot,iDot]=turret_plant(t,V,theta,thetaDot,current);
                theta=theta+dt*thetaRate;
                thetaDot=thetaDot+dt*thetaDDot;
                current=current+dt*iDot;
                if theta>=85*pi/180, theta=85*pi/180; thetaDot=min(thetaDot,0); end
                if theta<=-10*pi/180, theta=-10*pi/180; thetaDot=max(thetaDot,0); end
            end
        end
        use=tsim>=1;
        scenario{end+1,1}=char(d.id); run_id(end+1,1)=m;
        est_rmse(end+1,1)=sqrt(mean(ee(kEval)));
        pred_rmse(end+1,1)=sqrt(mean(ep(kEval)));
        los_rmse_deg(end+1,1)=180/pi*sqrt(mean(wrap(thLog(use)-losTrue(use)).^2));
        max_voltage(end+1,1)=max(abs(vLog)); pulse_count(end+1,1)=pulses;
        success(end+1,1)=hit; first_hit_s(end+1,1)=firstHit;
        if isinf(bestMiss), bestMiss=NaN; end, best_miss_m(end+1,1)=bestMiss;

        if m==1
            f=figure('Visible','off','Position',[100 100 1200 700]);
            subplot(2,2,1); plot(d.x(1,1:N),d.x(2,1:N),'k',xh(1,:),xh(2,:),'b--',xp(1,:),xp(2,:),'r:');
            axis equal; grid on; xlabel('x [m]'); ylabel('y [m]'); legend('truth','estimate','0.5 s prediction','Location','best'); title(['Scenario ' char(d.id)]);
            subplot(2,2,2); plot(tsim,180/pi*losTrue,'k',tsim,180/pi*thLog,'b--'); grid on; xlabel('t [s]'); ylabel('angle [deg]'); legend('current true LOS','turret');
            subplot(2,2,3); plot(tmeas,sqrt(ee),'b',tmeas,sqrt(ep),'r'); grid on; xlabel('t [s]'); ylabel('position error [m]'); legend('estimate','0.5 s prediction');
            subplot(2,2,4); miss=ray_miss(hypot(xtrue,ytrue),thLog-losTrue); plot(tsim,miss,'k'); hold on; yline(2,'r--');
            plot(tsim(fireLog),miss(fireLog),'go'); ylim([0 min(10,max(3,max(miss)))]); grid on; xlabel('t [s]'); ylabel('ray miss [m]'); legend('miss distance','2 m limit','pulses');
            style_figure(f); exportgraphics(f,fullfile(root,'figures',['matlab_scenario_' char(d.id) '.png']),'Resolution',160); close(f);
        end
    end
    fprintf('Scenario %s complete: %d noise runs\n',char(d.id),nUse);
end

runs=table(scenario,run_id,est_rmse,pred_rmse,los_rmse_deg,max_voltage,pulse_count,success,first_hit_s,best_miss_m);
writetable(runs,fullfile(root,'results','matlab_standardized_runs.csv'));

sc={'A';'B';'C'}; nr=zeros(3,1); success_rate=zeros(3,1); mean_est_rmse_m=zeros(3,1);
mean_pred_rmse_m=zeros(3,1); mean_los_rmse_deg=zeros(3,1); median_first_hit_s=zeros(3,1); worst_best_miss_m=zeros(3,1);
for j=1:3
    idx=strcmp(scenario,sc{j}); nr(j)=sum(idx); success_rate(j)=mean(success(idx));
    mean_est_rmse_m(j)=mean(est_rmse(idx)); mean_pred_rmse_m(j)=mean(pred_rmse(idx));
    mean_los_rmse_deg(j)=mean(los_rmse_deg(idx)); median_first_hit_s(j)=median(first_hit_s(idx),'omitnan');
    worst_best_miss_m(j)=max(best_miss_m(idx),[],'omitnan');
end
summary=table(sc,nr,success_rate,mean_est_rmse_m,mean_pred_rmse_m,mean_los_rmse_deg,median_first_hit_s,worst_best_miss_m,...
    'VariableNames',{'scenario','runs','success_rate','mean_est_rmse_m','mean_pred_rmse_m','mean_los_rmse_deg','median_first_hit_s','worst_best_miss_m'});
writetable(summary,fullfile(root,'results','matlab_standardized_summary.csv'));
disp(summary);
end

function a=wrap(a)
a=atan2(sin(a),cos(a));
end

function m=ray_miss(r,delta)
% Minimum distance to the forward ray, not to the infinite beam line.
delta=atan2(sin(delta),cos(delta));
m=r;
forward=cos(delta)>=0;
m(forward)=r(forward).*abs(sin(delta(forward)));
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
