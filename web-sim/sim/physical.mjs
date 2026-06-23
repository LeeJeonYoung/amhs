// ───────────────────────────────────────────────────────────────────────────
// 물리 모델 vs 추상(BPR) 모델 비교
//   추상(abstract): 혼잡을 "주행시간 증가(BPR)"로 모델링 → 데드락 없음 (우리 index.html 방식)
//   물리(physical): 노드 점유 + 하드 blocking + 데드락 감지/회복 (상용툴 AutoMod/AnyLogic 방식)
//   질문: "뭐가 더 좋은가?" → 밀도별로 throughput·완료율·데드락을 측정해 답한다.
//   실행: node sim/physical.mjs
// ───────────────────────────────────────────────────────────────────────────

function mulberry32(a){return function(){a|=0;a=a+0x6D2B79F5|0;let t=Math.imul(a^a>>>15,1|a);
  t=t+Math.imul(t^t>>>7,61|t)^t;return((t^t>>>14)>>>0)/4294967296;};}
function poisson(rng,l){if(l<=0)return 0;const L=Math.exp(-l);let k=0,p=1;do{k++;p*=rng();}while(p>L);return k-1;}

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
  return {rows,cols,el,nodes,adj,edgeLen,stations:nodes.map((_,i)=>i),length:(u,v)=>edgeLen[u+","+v]};
}
function dijkstra(g,src,dst,weight,blocked){
  weight=weight||((u,v)=>g.length(u,v));blocked=blocked||null;
  const dist={},prev={},visited={};dist[src]=0;const pq=[[0,src]];
  while(pq.length){pq.sort((a,b)=>a[0]-b[0]);const [d,u]=pq.shift();
    if(visited[u])continue;visited[u]=1;if(u===dst)break;
    for(const v of g.adj[u]){
      if(blocked&&blocked.has(v)&&v!==dst)continue;
      const nd=d+weight(u,v);
      if(dist[v]===undefined||nd<dist[v]){dist[v]=nd;prev[v]=u;pq.push([nd,v]);}}}
  if(dist[dst]===undefined)return [];
  const path=[dst];while(path[path.length-1]!==src)path.push(prev[path[path.length-1]]);
  return path.reverse();
}
function allPairs(g){const n=g.nodes.length,D=[];
  for(let s=0;s<n;s++){const dist=new Array(n).fill(Infinity);dist[s]=0;
    const pq=[[0,s]],seen=new Array(n).fill(false);
    while(pq.length){pq.sort((a,b)=>a[0]-b[0]);const [d,u]=pq.shift();
      if(seen[u])continue;seen[u]=true;
      for(const v of g.adj[u]){const nd=d+g.length(u,v);if(nd<dist[v]){dist[v]=nd;pq.push([nd,v]);}}}
    D.push(dist);}return D;}
function hungarian(cost){
  const n=cost.length;if(n===0)return [];const m=cost[0].length;const INF=Infinity;
  const u=new Array(n+1).fill(0),v=new Array(m+1).fill(0),p=new Array(m+1).fill(0),way=new Array(m+1).fill(0);
  for(let i=1;i<=n;i++){p[0]=i;let j0=0;const minv=new Array(m+1).fill(INF),used=new Array(m+1).fill(false);
    do{used[j0]=true;const i0=p[j0];let delta=INF,j1=-1;
      for(let j=1;j<=m;j++)if(!used[j]){const cur=cost[i0-1][j-1]-u[i0]-v[j];
        if(cur<minv[j]){minv[j]=cur;way[j]=j0;}if(minv[j]<delta){delta=minv[j];j1=j;}}
      for(let j=0;j<=m;j++){if(used[j]){u[p[j]]+=delta;v[j]-=delta;}else minv[j]-=delta;}j0=j1;
    }while(p[j0]!==0);
    do{const j1=way[j0];p[j0]=p[j1];j0=j1;}while(j0);}
  const res=new Array(n).fill(-1);for(let j=1;j<=m;j++)if(p[j]>0&&p[j]<=n)res[p[j]-1]=j-1;return res;}

const CFG={rows:4,cols:7,el:10,speed:2,pHot:0.1,loadTicks:5,unloadTicks:5,
  alpha:0.5,beta:0.6,w1:0.5,hotBonus:1000,window:30,
  deadlockThresh:8};   // 우회도 실패하고 이만큼 연속 blocked면 데드락→강제통과
const VST={IDLE:0,TO_SRC:1,LOADING:2,TO_DST:3,UNLOADING:4};

class Sim{
  // mode: "abstract" (BPR) | "physical" (노드점유+blocking+데드락)
  constructor(seed,rate,nVeh,mode){
    this.g=makeGrid(CFG.rows,CFG.cols,CFG.el);this.D=allPairs(this.g);
    this.rng=mulberry32(seed>>>0);this.rate=rate;this.nVeh=nVeh;this.mode=mode;
    this.now=0;this.nextId=0;this.pending=[];this.edgeLoad={};
    this.completed=0;this.sumCyc=0;this.assigned=0;this.sumQ=0;this.generated=0;
    this.blockedTicks=0;this.deadlocks=0;this.reroutes=0;
    this.occ={}; // 물리: node -> vehicle id (점유)
    const S=this.g.stations,step=Math.max(1,Math.floor(S.length/nVeh));
    this.veh=[];
    for(let i=0;i<nVeh;i++){
      const pos=S[(i*step)%S.length];
      const v={id:i,pos,state:VST.IDLE,task:null,dest:null,path:[],tr:0,proc:0,
        curEdge:null,blocked:0,holding:[pos]};
      this.veh.push(v);
      if(this.mode==="physical")this.occ[pos]=i; // 시작 노드 점유
    }
  }
  edgeCount(u,v){return (this.edgeLoad[u+","+v]||0)+(this.edgeLoad[v+","+u]||0);}
  // 추상은 혼잡 인지 라우팅, 물리는 정적(혼잡이 물리적으로 발생)
  route(src,dst,blocked){
    if(this.mode==="abstract"){
      const a=CFG.alpha,el=this.edgeLoad,g=this.g;
      const w=(u,v)=>g.length(u,v)*(1+a*((el[u+","+v]||0)+(el[v+","+u]||0)));
      const p=dijkstra(g,src,dst,w);return p.length>1?p.slice(1):[];
    }
    const p=dijkstra(this.g,src,dst,null,blocked);return p.length>1?p.slice(1):[];
  }
  genTasks(){
    const k=poisson(this.rng,this.rate),S=this.g.stations;
    for(let i=0;i<k;i++){let a=Math.floor(this.rng()*S.length),b=Math.floor(this.rng()*(S.length-1));
      if(b>=a)b++;const hot=this.rng()<CFG.pHot;
      this.pending.push({id:this.nextId++,src:S[a],dst:S[b],hot,created:this.now,assigned:-1});
      this.generated++;}
  }
  // ── 이동 ──
  occBlockedSet(self,dest){ // 점유 노드 집합(자기 위치·목적지 제외) — 우회 라우팅용
    const s=new Set();for(const k in this.occ){const id=this.occ[k],node=+k;
      if(id!==self&&node!==dest)s.add(node);}return s;}
  tryStartHop(v){
    let nxt=v.path[0];
    if(this.mode==="physical"){
      // 다음 노드가 비었으면 진입
      if(this.occ[nxt]===undefined||this.occ[nxt]===v.id){
        this.occ[nxt]=v.id;
        const ff=Math.max(1,Math.ceil(this.g.length(v.pos,nxt)/CFG.speed));
        v.tr=ff;v.curEdge=[v.pos,nxt];v.blocked=0;return true;
      }
      // 막힘 → 점유 노드 회피 동적 재경로 시도(목적지는 허용=포트 대기)
      const alt=this.route(v.pos,v.dest,this.occBlockedSet(v.id,v.dest));
      if(alt.length&&(this.occ[alt[0]]===undefined||this.occ[alt[0]]===v.id)){
        v.path=alt;nxt=v.path[0];this.reroutes++;
        this.occ[nxt]=v.id;
        const ff=Math.max(1,Math.ceil(this.g.length(v.pos,nxt)/CFG.speed));
        v.tr=ff;v.curEdge=[v.pos,nxt];v.blocked=0;return true;
      }
      // 우회도 실패 → 대기(blocking)
      v.blocked++;this.blockedTicks++;return false;
    }else{
      const load=this.edgeCount(v.pos,nxt);
      const ff=Math.max(1,Math.ceil(this.g.length(v.pos,nxt)/CFG.speed));
      const tt=Math.max(1,Math.ceil(ff*(1+CFG.beta*load)));
      v.tr=tt;const e=v.pos+","+nxt;this.edgeLoad[e]=(this.edgeLoad[e]||0)+1;v.curEdge=[v.pos,nxt];return true;
    }
  }
  arrive(v){
    const old=v.pos;v.pos=v.path.shift();
    if(this.mode==="physical"){if(this.occ[old]===v.id)delete this.occ[old];this.occ[v.pos]=v.id;v.curEdge=null;}
    else{if(v.curEdge){const e=v.curEdge[0]+","+v.curEdge[1];this.edgeLoad[e]--;v.curEdge=null;}}
  }
  moveStep(v){
    if(v.tr===0&&v.path.length)this.tryStartHop(v);
    if(v.tr>0){v.tr--;if(v.tr===0)this.arrive(v);}
  }
  phase(v){
    if(v.tr!==0||v.path.length)return;
    if(v.state===VST.TO_SRC){v.state=VST.LOADING;v.proc=CFG.loadTicks;}
    else if(v.state===VST.TO_DST){v.state=VST.UNLOADING;v.proc=CFG.unloadTicks;}
  }
  procStep(v){
    if(--v.proc>0)return;
    if(v.state===VST.LOADING){v.dest=v.task.dst;v.path=this.route(v.pos,v.dest);v.state=VST.TO_DST;this.phase(v);}
    else if(v.state===VST.UNLOADING){const cyc=this.now-v.task.created;this.completed++;this.sumCyc+=cyc;
      v.task=null;v.dest=null;v.path=[];v.state=VST.IDLE;}
  }
  // ── 데드락 감지/회복 (물리 전용) ──
  recoverDeadlocks(){
    // 우회(이동 단계)도 실패하고 thresh 넘게 막힌 경우 = 진짜 교착 → 강제 통과 1회(수동 개입 대리)
    for(const v of this.veh){
      if(v.blocked>CFG.deadlockThresh && v.path.length){
        this.deadlocks++;
        const nxt=v.path[0];
        if(this.occ[nxt]!==undefined&&this.occ[nxt]!==v.id)delete this.occ[nxt];
        v.blocked=0;
      }
    }
  }
  candidates(){return this.pending.slice()
    .sort((a,b)=>(b.hot-a.hot)||(a.created-b.created)||(a.id-b.id)).slice(0,CFG.window);}
  dispatch(){
    const idle=this.veh.filter(v=>v.state===VST.IDLE);
    if(!idle.length||!this.pending.length)return;
    const cand=this.candidates(),n=idle.length,m=cand.length,BIG=1e7,cols=Math.max(n,m),cost=[];
    for(let i=0;i<n;i++){const row=[];
      for(let j=0;j<cols;j++){
        if(j<m){const t=cand[j];let c=this.D[idle[i].pos][t.src]-CFG.w1*(this.now-t.created);
          if(t.hot)c-=CFG.hotBonus;row.push(c);}else row.push(BIG);}
      cost.push(row);}
    const a=hungarian(cost),pairs=[];
    for(let i=0;i<n;i++){const j=a[i];if(j>=0&&j<m)pairs.push([idle[i],cand[j]]);}
    for(const [v,t] of pairs){v.task=t;v.dest=t.src;v.state=VST.TO_SRC;t.assigned=this.now;
      this.sumQ+=(this.now-t.created);this.assigned++;v.path=this.route(v.pos,t.src);
      const idx=this.pending.indexOf(t);if(idx>=0)this.pending.splice(idx,1);this.phase(v);}
  }
  step(){
    this.genTasks();
    for(const v of this.veh){ // id 순서 → 결정론적 예약
      if(v.state===VST.TO_SRC||v.state===VST.TO_DST){this.moveStep(v);this.phase(v);}
      else if(v.state===VST.LOADING||v.state===VST.UNLOADING)this.procStep(v);
    }
    if(this.mode==="physical")this.recoverDeadlocks();
    this.dispatch();
    this.now++;
  }
  kpis(){const T=Math.max(1,this.now);return{
    throughput_per_1k:this.completed/T*1000,
    avg_cycle_time:this.completed?this.sumCyc/this.completed:0,
    avg_queue_wait:this.assigned?this.sumQ/this.assigned:0,
    completion_rate:this.generated?this.completed/this.generated:0,
    deadlocks:this.deadlocks,
    reroutes:this.reroutes,
    block_ratio:this.blockedTicks/(this.nVeh*T),
  };}
}

// ── 실행: 차량 밀도를 올려가며 추상 vs 물리 비교 ──
const SEEDS=[0,1,2,3,4,5,6,7,8,9];
const T=2500;
const RATE=0.30;                      // 고부하 고정
const FLEETS=[6,10,14,18];            // 28노드 그리드에서 밀도 21%→64% (현실적 범위)
function run(seed,nVeh,mode){const s=new Sim(seed,RATE,nVeh,mode);for(let t=0;t<T;t++)s.step();return s.kpis();}
function meanOver(nVeh,mode){const acc={};for(const seed of SEEDS){const k=run(seed,nVeh,mode);
  for(const key in k)acc[key]=(acc[key]||0)+k[key];}for(const key in acc)acc[key]/=SEEDS.length;return acc;}
function fmt(x){return (Math.round(x*1000)/1000).toString();}

console.log("# 물리 모델 vs 추상(BPR) 모델 비교");
console.log(`- seeds 10, T=${T}, grid 4x7(28노드), rate=${RATE}, dispatch=Hungarian`);
console.log("- abstract: 혼잡=BPR 주행시간 증가(데드락 없음) | physical: 노드점유+하드 blocking+데드락 감지/회복\n");
const cols=["throughput_per_1k","avg_cycle_time","avg_queue_wait","completion_rate","deadlocks","reroutes","block_ratio"];
for(const mode of ["abstract","physical"]){
  console.log(`## ${mode}`);
  console.log("| vehicles | "+cols.join(" | ")+" |");
  console.log("|"+"---|".repeat(cols.length+1));
  for(const nv of FLEETS){const r=meanOver(nv,mode);
    console.log(`| ${nv} | `+cols.map(c=>fmt(r[c])).join(" | ")+" |");}
  console.log("");
}
// 직접 비교: 같은 차량 수에서 물리가 추상 대비 처리량 몇 % 손실?
console.log("## 직접 비교 (physical이 abstract 대비 throughput 손실 %)");
console.log("| vehicles | abstract thr | physical thr | 손실% | physical 데드락 |");
console.log("|---|---|---|---|---|");
for(const nv of FLEETS){const a=meanOver(nv,"abstract"),p=meanOver(nv,"physical");
  const loss=(a.throughput_per_1k-p.throughput_per_1k)/a.throughput_per_1k*100;
  console.log(`| ${nv} | ${fmt(a.throughput_per_1k)} | ${fmt(p.throughput_per_1k)} | ${fmt(loss)}% | ${fmt(p.deadlocks)} |`);}
console.log("\n(끝)");
