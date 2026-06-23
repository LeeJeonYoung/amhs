// ───────────────────────────────────────────────────────────────────────────
// mini-AMHS 실험 하네스 (Node) — index.html의 시뮬 코어를 그대로 포팅 + 현실 변수 4종
//   목적: "동일 모델" 위에 현실 변수를 얹어, Hungarian+congestion 우위가
//         실제에 가까운 조건(고장·충전·가변 처리·버스트 도착)에서도 유지되는지 측정.
//   실행: node sim/harness.mjs
// ───────────────────────────────────────────────────────────────────────────

// ── RNG / 분포 (index.html과 동일) ──
function mulberry32(a){return function(){a|=0;a=a+0x6D2B79F5|0;let t=Math.imul(a^a>>>15,1|a);
  t=t+Math.imul(t^t>>>7,61|t)^t;return((t^t>>>14)>>>0)/4294967296;};}
function poisson(rng,l){if(l<=0)return 0;const L=Math.exp(-l);let k=0,p=1;do{k++;p*=rng();}while(p>L);return k-1;}

// ── 그래프 / Dijkstra / allPairs (동일) ──
function makeGrid(rows,cols,el){
  const nodes=[],adj=[],edgeLen={};
  const nid=(r,c)=>r*cols+c;
  for(let r=0;r<rows;r++)for(let c=0;c<cols;c++){nodes.push({r,c});adj.push([]);}
  const conn=(a,b)=>{if(!adj[a].includes(b)){adj[a].push(b);edgeLen[a+","+b]=el;}
                     if(!adj[b].includes(a)){adj[b].push(a);edgeLen[b+","+a]=el;}};
  for(let r=0;r<rows;r++)for(let c=0;c<cols;c++){
    if(c+1<cols)conn(nid(r,c),nid(r,c+1));
    if(r+1<rows)conn(nid(r,c),nid(r+1,c));
  }
  const stations=nodes.map((_,i)=>i);
  return {rows,cols,el,nodes,adj,edgeLen,stations,length:(u,v)=>edgeLen[u+","+v]};
}
function dijkstra(g,src,dst,weight){
  weight=weight||((u,v)=>g.length(u,v));
  const dist={},prev={},visited={};
  dist[src]=0;const pq=[[0,src]];
  while(pq.length){
    pq.sort((a,b)=>a[0]-b[0]);
    const [d,u]=pq.shift();
    if(visited[u])continue;visited[u]=1;
    if(u===dst)break;
    for(const v of g.adj[u]){
      const nd=d+weight(u,v);
      if(dist[v]===undefined||nd<dist[v]){dist[v]=nd;prev[v]=u;pq.push([nd,v]);}
    }
  }
  if(dist[dst]===undefined)return [];
  const path=[dst];while(path[path.length-1]!==src)path.push(prev[path[path.length-1]]);
  return path.reverse();
}
function allPairs(g){
  const n=g.nodes.length,D=[];
  for(let s=0;s<n;s++){
    const dist=new Array(n).fill(Infinity);dist[s]=0;
    const pq=[[0,s]],seen=new Array(n).fill(false);
    while(pq.length){pq.sort((a,b)=>a[0]-b[0]);const [d,u]=pq.shift();
      if(seen[u])continue;seen[u]=true;
      for(const v of g.adj[u]){const nd=d+g.length(u,v);if(nd<dist[v]){dist[v]=nd;pq.push([nd,v]);}}}
    D.push(dist);
  }
  return D;
}

// ── Hungarian (손구현 Kuhn–Munkres, index.html과 동일) ──
function hungarian(cost){
  const n=cost.length;if(n===0)return [];
  const m=cost[0].length;const INF=Infinity;
  const u=new Array(n+1).fill(0),v=new Array(m+1).fill(0),
        p=new Array(m+1).fill(0),way=new Array(m+1).fill(0);
  for(let i=1;i<=n;i++){
    p[0]=i;let j0=0;
    const minv=new Array(m+1).fill(INF),used=new Array(m+1).fill(false);
    do{
      used[j0]=true;const i0=p[j0];let delta=INF,j1=-1;
      for(let j=1;j<=m;j++)if(!used[j]){
        const cur=cost[i0-1][j-1]-u[i0]-v[j];
        if(cur<minv[j]){minv[j]=cur;way[j]=j0;}
        if(minv[j]<delta){delta=minv[j];j1=j;}
      }
      for(let j=0;j<=m;j++){if(used[j]){u[p[j]]+=delta;v[j]-=delta;}else minv[j]-=delta;}
      j0=j1;
    }while(p[j0]!==0);
    do{const j1=way[j0];p[j0]=p[j1];j0=j1;}while(j0);
  }
  const res=new Array(n).fill(-1);
  for(let j=1;j<=m;j++)if(p[j]>0&&p[j]<=n)res[p[j]-1]=j-1;
  return res;
}

// ── 기본 파라미터 (index.html CFG와 동일) ──
const BASE={rows:4,cols:7,el:10,speed:2,vehicles:8,pHot:0.1,
  loadTicks:5,unloadTicks:5,alpha:0.5,beta:0.6,w1:0.5,hotBonus:1000,window:30};
const VST={IDLE:0,TO_SRC:1,LOADING:2,TO_DST:3,UNLOADING:4,TO_CHARGE:5,CHARGING:6,DOWN:7};

class Sim{
  // mods: 현실 변수 옵션 (전부 off면 index.html과 동일 동작)
  constructor(seed,rate,disp,rout,cfg,mods){
    this.cfg=cfg;this.mods=mods||{};
    this.g=makeGrid(cfg.rows,cfg.cols,cfg.el);
    this.D=allPairs(this.g);
    this.rng=mulberry32(seed>>>0);
    this.rate=rate;this.disp=disp;this.rout=rout;
    this.now=0;this.nextId=0;this.pending=[];this.edgeLoad={};
    this.completed=0;this.sumCyc=0;this.assigned=0;this.sumQ=0;
    this.busy=0;this.ff=0;this.cong=0;this.generated=0;
    this.downTicks=0;this.dropped=0;this.chargeTicks=0; // 현실변수 KPI
    const S=this.g.stations,step=Math.max(1,Math.floor(S.length/cfg.vehicles));
    this.veh=[];
    for(let i=0;i<cfg.vehicles;i++){
      const pos=S[(i*step)%S.length];
      this.veh.push({id:i,pos,state:VST.IDLE,task:null,dest:null,path:[],
        tr:0,curEdge:null,proc:0,batt:(this.mods.battery?this.mods.battery.capacity:Infinity),down:0});
    }
    // 충전소: 네 모서리
    this.chargers=[0,cfg.cols-1,(cfg.rows-1)*cfg.cols,cfg.rows*cfg.cols-1];
  }
  edgeCount(u,v){return (this.edgeLoad[u+","+v]||0)+(this.edgeLoad[v+","+u]||0);}
  route(src,dst){
    if(this.rout==="congestion"){
      const a=this.cfg.alpha,el=this.edgeLoad,g=this.g;
      const w=(u,v)=>g.length(u,v)*(1+a*((el[u+","+v]||0)+(el[v+","+u]||0)));
      const p=dijkstra(g,src,dst,w);return p.length>1?p.slice(1):[];
    }
    const p=dijkstra(this.g,src,dst);return p.length>1?p.slice(1):[];
  }
  nearestCharger(pos){let best=this.chargers[0],bd=this.D[pos][best];
    for(const c of this.chargers){const d=this.D[pos][c];if(d<bd){bd=d;best=c;}}return best;}

  genTasks(){
    const S=this.g.stations;
    let k;
    const burst=this.mods.burst;
    if(burst&&burst.enabled){
      // MES 로트 릴리스: period마다 batch개를 한꺼번에 투입 (평균 도착률은 rate와 동일하게)
      k=(this.now%burst.period===0)?burst.batch:0;
    }else{
      k=poisson(this.rng,this.rate);
    }
    for(let i=0;i<k;i++){
      let a=Math.floor(this.rng()*S.length),b=Math.floor(this.rng()*(S.length-1));
      if(b>=a)b++;
      const hot=this.rng()<this.cfg.pHot;
      this.pending.push({id:this.nextId++,src:S[a],dst:S[b],hot,created:this.now,assigned:-1});
      this.generated++;
    }
  }
  procTicks(mean){
    // 가변 처리시간: mean 중심 균등 [0.4, 1.6]*mean (off면 고정 mean)
    if(this.mods.stochasticHandling&&this.mods.stochasticHandling.enabled){
      return Math.max(1,Math.round(mean*(0.4+1.2*this.rng())));
    }
    return mean;
  }
  startHop(v){
    const nxt=v.path[0],load=this.edgeCount(v.pos,nxt);
    const ff=Math.max(1,Math.ceil(this.g.length(v.pos,nxt)/this.cfg.speed));
    const tt=Math.max(1,Math.ceil(ff*(1+this.cfg.beta*load)));
    v.tr=tt;this.ff+=ff;this.cong+=(tt-ff);
    const e=v.pos+","+nxt;this.edgeLoad[e]=(this.edgeLoad[e]||0)+1;v.curEdge=[v.pos,nxt];
  }
  arrive(v){
    if(v.curEdge){const e=v.curEdge[0]+","+v.curEdge[1];this.edgeLoad[e]--;v.curEdge=null;}
    v.pos=v.path.shift();
    if(this.mods.battery&&this.mods.battery.enabled&&v.state!==VST.TO_CHARGE)
      v.batt-=this.mods.battery.drainPerHop;
  }
  moveStep(v){
    if(v.tr===0&&v.path.length)this.startHop(v);
    if(v.tr>0){v.tr--;if(v.tr===0)this.arrive(v);}
  }
  phase(v){
    if(v.tr!==0||v.path.length)return;
    if(v.state===VST.TO_SRC){v.state=VST.LOADING;v.proc=this.procTicks(this.cfg.loadTicks);}
    else if(v.state===VST.TO_DST){v.state=VST.UNLOADING;v.proc=this.procTicks(this.cfg.unloadTicks);}
    else if(v.state===VST.TO_CHARGE){v.state=VST.CHARGING;
      const b=this.mods.battery;v.proc=Math.max(1,Math.ceil((b.capacity-v.batt)/b.chargeRate));}
  }
  procStep(v){
    if(v.state===VST.CHARGING){this.chargeTicks++;if(--v.proc>0)return;
      v.batt=this.mods.battery.capacity;v.state=VST.IDLE;return;}
    if(--v.proc>0)return;
    if(v.state===VST.LOADING){v.dest=v.task.dst;v.path=this.route(v.pos,v.dest);v.state=VST.TO_DST;this.phase(v);}
    else if(v.state===VST.UNLOADING){
      const cyc=this.now-v.task.created;this.completed++;this.sumCyc+=cyc;
      v.task=null;v.dest=null;v.path=[];v.state=VST.IDLE;
    }
  }
  maybeBreakdown(v){
    const b=this.mods.breakdown;if(!b||!b.enabled)return false;
    if(v.down>0){v.down--;this.downTicks++;return true;}
    // 노드에서만(이동 중 아님) 고장 발생 → 트랙 한가운데 blocking 회피(혼잡 추상화와 분리)
    if(v.tr===0&&this.rng()<1/b.mtbf){
      // 지수분포 수리시간
      v.down=Math.max(1,Math.ceil(-Math.log(1-this.rng())*b.mttr));
      this.downTicks++;v.down--;return true;
    }
    return false;
  }
  candidates(){
    const arr=this.pending.slice().sort((a,b)=>(b.hot-a.hot)||(a.created-b.created)||(a.id-b.id));
    return arr.slice(0,this.cfg.window);
  }
  dispatchable(v){return v.state===VST.IDLE&&v.down===0&&
    !(this.mods.battery&&this.mods.battery.enabled&&v.batt<=this.mods.battery.threshold);}
  dispatch(){
    const idle=this.veh.filter(v=>this.dispatchable(v));
    if(!idle.length||!this.pending.length)return;
    const cand=this.candidates();
    let pairs=[];
    if(this.disp==="greedy"){
      const pool=idle.slice();
      for(const t of cand){
        if(!pool.length)break;
        let best=pool[0],bd=this.D[best.pos][t.src];
        for(const v of pool){const d=this.D[v.pos][t.src];if(d<bd||(d===bd&&v.id<best.id)){best=v;bd=d;}}
        pairs.push([best,t]);pool.splice(pool.indexOf(best),1);
      }
    }else{
      const n=idle.length,m=cand.length,BIG=1e7,cols=Math.max(n,m),cost=[];
      for(let i=0;i<n;i++){const row=[];
        for(let j=0;j<cols;j++){
          if(j<m){const t=cand[j];let c=this.D[idle[i].pos][t.src]-this.cfg.w1*(this.now-t.created);
            if(t.hot)c-=this.cfg.hotBonus;row.push(c);}
          else row.push(BIG);}
        cost.push(row);}
      const a=hungarian(cost);
      for(let i=0;i<n;i++){const j=a[i];if(j>=0&&j<m)pairs.push([idle[i],cand[j]]);}
    }
    for(const [v,t] of pairs){
      v.task=t;v.dest=t.src;v.state=VST.TO_SRC;t.assigned=this.now;
      this.sumQ+=(this.now-t.created);this.assigned++;
      v.path=this.route(v.pos,t.src);
      const idx=this.pending.indexOf(t);if(idx>=0)this.pending.splice(idx,1);
      this.phase(v);
    }
  }
  sendToCharge(){
    if(!this.mods.battery||!this.mods.battery.enabled)return;
    for(const v of this.veh){
      if(v.state===VST.IDLE&&v.down===0&&v.batt<=this.mods.battery.threshold){
        const c=this.nearestCharger(v.pos);
        v.state=VST.TO_CHARGE;v.path=this.route(v.pos,c);this.phase(v);
      }
    }
  }
  step(){
    this.genTasks();
    for(const v of this.veh){
      if(this.maybeBreakdown(v))continue; // 고장 중이면 아무것도 안 함
      if(v.state===VST.TO_SRC||v.state===VST.TO_DST||v.state===VST.TO_CHARGE){this.moveStep(v);this.phase(v);}
      else if(v.state===VST.LOADING||v.state===VST.UNLOADING||v.state===VST.CHARGING)this.procStep(v);
    }
    this.sendToCharge();
    this.dispatch();
    for(const v of this.veh)if(v.state!==VST.IDLE&&v.down===0)this.busy++;
    this.now++;
  }
  kpis(){
    const T=Math.max(1,this.now),N=this.cfg.vehicles;
    return {
      throughput_per_1k:this.completed/T*1000,
      avg_cycle_time:this.completed?this.sumCyc/this.completed:0,
      avg_queue_wait:this.assigned?this.sumQ/this.assigned:0,
      congestion_delay_ratio:this.ff?this.cong/this.ff:0,
      completion_rate:this.generated?this.completed/this.generated:0,
      utilization:this.busy/(N*T),
      down_ratio:this.downTicks/(N*T),
      charge_ratio:this.chargeTicks/(N*T),
    };
  }
}

// ── 실행 ──
const SEEDS=[0,1,2,3,4,5,6,7,8,9];
const T=2500;
const CONFIGS=[["greedy","static"],["greedy","congestion"],["hungarian","static"],["hungarian","congestion"]];
const LOADS={moderate:0.18, heavy:0.30};

function run(rate,disp,rout,cfg,mods,seed){
  const s=new Sim(seed,rate,disp,rout,cfg,mods);
  for(let t=0;t<T;t++)s.step();
  return s.kpis();
}
function meanOver(rate,disp,rout,cfg,mods){
  const acc={};let first=true;
  for(const seed of SEEDS){
    const k=run(rate,disp,rout,cfg,mods,seed);
    for(const key in k){acc[key]=(acc[key]||0)+k[key];}
  }
  for(const key in acc)acc[key]/=SEEDS.length;
  return acc;
}
function pct(a,b){return ((b-a)/a*100);}
function fmt(x){return (Math.round(x*1000)/1000).toString();}

function table(scenarioName,rate,cfg,mods){
  const rows={};
  for(const [d,r] of CONFIGS)rows[`${d}+${r}`]=meanOver(rate,d,r,cfg,mods);
  const base=rows["greedy+static"],best=rows["hungarian+congestion"];
  return {scenarioName,rows,headline:{
    cycle_time_change_pct:pct(base.avg_cycle_time,best.avg_cycle_time),
    queue_wait_change_pct:pct(base.avg_queue_wait,best.avg_queue_wait),
    throughput_change_pct:pct(base.throughput_per_1k,best.throughput_per_1k),
    congestion_change_pct:pct(base.congestion_delay_ratio,best.congestion_delay_ratio),
    completion_base:base.completion_rate, completion_best:best.completion_rate,
  }};
}

function printTable(t){
  console.log(`\n### ${t.scenarioName}`);
  const cols=["throughput_per_1k","avg_cycle_time","avg_queue_wait","congestion_delay_ratio","completion_rate","utilization","down_ratio","charge_ratio"];
  console.log("| config | "+cols.join(" | ")+" |");
  console.log("|"+"---|".repeat(cols.length+1));
  for(const name in t.rows){
    const r=t.rows[name];
    console.log(`| ${name} | `+cols.map(c=>fmt(r[c])).join(" | ")+" |");
  }
  const h=t.headline;
  console.log(`**Headline (hungarian+congestion vs greedy+static):** cycle ${fmt(h.cycle_time_change_pct)}% · queue_wait ${fmt(h.queue_wait_change_pct)}% · throughput +${fmt(h.throughput_change_pct)}% · 완료율 ${fmt(h.completion_base)}→${fmt(h.completion_best)}`);
}

// 현실 변수 정의
const REAL_MODS={
  breakdown:{enabled:true,mtbf:1500,mttr:80},            // 평균 1500틱마다 고장, 평균 80틱 수리
  battery:{enabled:true,capacity:120,threshold:25,drainPerHop:1,chargeRate:6}, // 충전
  stochasticHandling:{enabled:true},                      // 적/하역 시간 변동
  burst:{enabled:true,period:1,batch:0},                  // (도착은 아래에서 별도 시나리오로)
};
// 도착 패턴은 별도 시나리오로 분리: REAL_MODS에서는 버스트 끔(Poisson 유지)
const REAL_NOBURST={...REAL_MODS,burst:{enabled:false}};

console.log("# mini-AMHS 현실 변수 실험 결과");
console.log(`- seeds: ${SEEDS.length}개, T=${T}틱/run, 그리드 ${BASE.rows}x${BASE.cols}, 차량 ${BASE.vehicles}대`);
console.log("\n## [A] 베이스라인 (현실변수 OFF — index.html과 동일 모델)");
for(const L in LOADS)printTable(table(`${L}_load (rate=${LOADS[L]})`,LOADS[L],BASE,{}));

console.log("\n## [B] 현실 변수 ON (고장+배터리+가변처리, Poisson 도착)");
for(const L in LOADS)printTable(table(`${L}_load (rate=${LOADS[L]})`,LOADS[L],BASE,REAL_NOBURST));

console.log("\n## [C] MES 버스트 도착 (로트 릴리스: 일정 주기에 일괄 투입) + 현실변수");
// 평균 도착률 유지: period*rate ≈ batch
for(const L in LOADS){
  const rate=LOADS[L];const period=120;const batch=Math.round(rate*period);
  const mods={...REAL_MODS,burst:{enabled:true,period,batch}};
  printTable(table(`${L}_burst (period=${period}, batch=${batch})`,rate,BASE,mods));
}

console.log("\n## [D] 단일 변수 ablation (heavy_load, hungarian+congestion만)");
const single={
  "변수없음":{}, "고장만":{breakdown:REAL_MODS.breakdown},
  "배터리만":{battery:REAL_MODS.battery}, "가변처리만":{stochasticHandling:{enabled:true}},
};
const acols=["throughput_per_1k","avg_cycle_time","avg_queue_wait","completion_rate","down_ratio","charge_ratio"];
console.log("| mod | "+acols.join(" | ")+" |");
console.log("|"+"---|".repeat(acols.length+1));
for(const name in single){
  const r=meanOver(LOADS.heavy,"hungarian","congestion",BASE,single[name]);
  console.log(`| ${name} | `+acols.map(c=>fmt(r[c])).join(" | ")+" |");
}
console.log("\n(끝)");
