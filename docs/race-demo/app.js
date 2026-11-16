(() => {
  'use strict';
  const $ = id => document.getElementById(id);
  const data = window.RaceData;
  const fixed = (n, digits=2) => Number.isFinite(n) ? n.toFixed(digits) : '—';
  const fmt = n => fixed(n) + ' s';
  const labels = {finished:'완주', gate_passed:'통과', collision:'충돌', out_of_bounds:'비행영역 이탈', reverse_gate:'게이트 역방향 통과', skipped_gate:'게이트 순서 위반', timeout:'비행시간 초과', invalid_action:'제어 명령 오류', controller_error:'제어기 오류', simulation_error:'시뮬레이션 오류'};
  const statusName = key => labels[key] || String(key || '종료');
  let runs=[], completed=new Set(), current=0, t=0, playing=false, ended=false, frameTime=null, framePaint=-1, view='3d', rate=1, imported=false, importSeq=0;
  let eventKey='', resumeOnClose=false;
  const cohortKeys = new WeakMap();
  function canonical(obj){
    if(Array.isArray(obj)) return obj.map(canonical);
    if(obj && typeof obj==='object')return Object.fromEntries(Object.keys(obj).sort().map(k=>[k,canonical(obj[k])]));
    return obj;
  }
  function cohort(r){
    if(!cohortKeys.has(r)){
      const l=r.log;
      cohortKeys.set(r,JSON.stringify(canonical([r.synthetic?'demo':'log',l.config,l.scenario,l.provenance?.python??null,l.provenance?.packages??null])));
    }
    return cohortKeys.get(r);
  }
  const run=()=>runs[current], log=()=>run().log;
  const total=()=>log().summary.flight_time_s;
  function indexAt(samples,time){
    let a=0,b=samples.length;
    while(a<b){const mid=Math.floor((a+b)/2);if(samples[mid].time_s<=time+1e-8)a=mid+1;else b=mid;}
    return Math.max(0,a-1);
  }
  function standings(){
    const rows=runs.filter(r=>completed.has(r.id)&&cohort(r)===cohort(run())).map(r=>({r,ok:r.log.summary.completed,ticks:Math.round(r.log.summary.score_s*100)}));
    rows.sort((a,b)=>Number(b.ok)-Number(a.ok)||(a.ok?a.ticks-b.ticks:0));
    let last=null,rank=0;rows.forEach((v,i)=>{if(v.ok&&v.ticks!==last){rank=i+1;last=v.ticks;}v.place=v.ok?rank:null;});
    return rows;
  }
  function board(){
    const rows=standings(), body=$('board');body.replaceChildren();
    for(const {r,ok,place} of rows){
      const s=r.log.summary,tr=document.createElement('tr');
      if(r.id===run().id&&ended)tr.classList.add('fresh');
      if(!ok)tr.classList.add('dnf');
      const dnf=s.status==='simulation_error'?'검토':'DNF';
      const cells=[place?String(place):'—',r.name,s.gates_passed+'/'+s.gates_total,fixed(s.flight_time_s), '+'+fixed(s.load_penalty_s),ok?fmt(s.score_s):dnf];
      cells.forEach((v,i)=>{
        const td=document.createElement('td');
        if(i===1){
          const replay=document.createElement('button');replay.className='replayName';replay.textContent=v+' ▷';replay.setAttribute('aria-label',v+' 비행 다시 보기');
          replay.onclick=()=>{select(runs.indexOf(r));resume();};td.append(replay);
        }else{const span=document.createElement('span');span.textContent=v;if(!i)span.className='rank'+(place===1?' lead':'');td.append(span);}
        tr.append(td);
      });body.append(tr);
    }
    if(!rows.length){const td=document.createElement('td'),tr=document.createElement('tr');td.colSpan=6;td.textContent='첫 비행이 끝나면 이곳에 기록이 표시됩니다.';tr.append(td);body.append(tr);}
    $('boardScope').textContent='동일 조건 · 종료 '+rows.length+'명';
    $('boardNote').textContent='참가자 이름을 눌러 다시 보기 · 기록 단위는 초(s) · 미완주(DNF)는 순위 제외';
  }
  function eventFeed(){
    const l=log(),visible=l.events.filter(e=>e.time_s<=t+1e-8 && (e.kind!=='finished'||ended));
    const key=[run().id,visible.length,ended].join('-');if(key===eventKey)return;eventKey=key;
    const entries=visible.map(e=>({name:e.kind==='gate_passed'?'G'+String(e.gate+1).padStart(2,'0')+' 게이트 통과':statusName(e.kind),time:e.time_s}));
    if(ended && !visible.some(e=>e.kind===l.summary.status))entries.push({name:statusName(l.summary.status),time:total()});
    entries.unshift({name:'출발선 · '+run().name,time:0});
    const list=$('events');list.replaceChildren();
    for(const e of entries.reverse()){const li=document.createElement('li'), tag=document.createElement('time');li.append(document.createTextNode(e.name));tag.textContent=fmt(e.time);li.append(tag);list.append(li);}
  }
  function chart(l,idx){
    const c=$('spark'),b=c.getBoundingClientRect(),dpi=Math.min(devicePixelRatio||1,2);
    if(!b.width||!b.height)return;
    const pw=Math.ceil(b.width*dpi),ph=Math.ceil(b.height*dpi);
    if(c.width!==pw||c.height!==ph){c.width=pw;c.height=ph;}
    const g=c.getContext('2d');g.setTransform(dpi,0,0,dpi,0,0);g.clearRect(0,0,b.width,b.height);
    const xs=l.samples,end=xs[idx].time_s,lo=Math.max(0,end-6),start=indexAt(xs,lo),limit=l.scenario.load_limit_g;
    const pts=[];const stride=Math.max(1,Math.ceil((idx-start)/90));
    for(let i=start;i<=idx;i+=stride){const v=data.metricsAt(l,i).load;if(Number.isFinite(v))pts.push([xs[i].time_s,v]);}
    const yMax=Math.max(limit*1.35,1,...pts.map(p=>p[1])),y=v=>b.height-2-(b.height-4)*v/yMax;
    g.strokeStyle='#aebdaf';g.setLineDash([4,4]);g.beginPath();g.moveTo(0,y(limit));g.lineTo(b.width,y(limit));g.stroke();g.setLineDash([]);
    g.strokeStyle='#138675';g.lineWidth=2;g.beginPath();pts.forEach((p,i)=>{const x=(p[0]-lo)/6*b.width;if(i===0)g.moveTo(x,y(p[1]));else g.lineTo(x,y(p[1]));});g.stroke();
  }
  function paint(){
    const r=run(),l=r.log,idx=indexAt(l.samples,t),s=l.samples[idx],m=data.metricsAt(l,idx),summary=l.summary;
    const preRecorded=completed.has(r.id)&&!ended;
    const state=ended?statusName(summary.status):playing?(preRecorded?'다시 보기':'비행 중'):t>0?'일시정지':'출발 대기';
    $('clock').textContent=fixed(t);$('statePill').textContent=state;$('courseStatus').textContent=state;
    $('velocity').textContent=fixed(m.speed,1);$('thrust').textContent=fixed(m.thrust,1);$('load').textContent=fixed(m.load,2);
    const gates=ended?summary.gates_passed:s.next_gate;
    $('gateCount').textContent='GATE '+String(gates).padStart(2,'0')+' / '+String(summary.gates_total).padStart(2,'0');
    $('scoreCaption').textContent=ended?(summary.completed?'최종 환산 기록':'미완주 진단값'):'현재 환산 기록 (추정)';
    $('score').textContent=fmt(ended?summary.score_s:t+(m.penalty||0));
    $('penalty').textContent=ended?'하중 가산 +'+fmt(summary.load_penalty_s)+'  ·  실패 가산 +'+fmt(summary.failure_penalty_s):'누적 하중 가산 추정 +'+fmt(m.penalty);
    $('scrub').value=total()>0?String(Math.round(t/total()*1000)):'0';
    $('play').textContent=ended?'↻  다시 재생':playing?'Ⅱ  일시정지':t>0?'▶  이어 재생':(imported?'▶  로그 재생':'▶  시연 시작');
    chart(l,idx);window.drawCourse($('courseCanvas'),l.config.track,l.samples,idx,l.events,view);eventFeed();
  }
  function finish(){
    t=total();playing=false;ended=true;completed.add(run().id);board();
    const s=log().summary,row=standings().find(x=>x.r.id===run().id);
    const rank=row?.place?'현재 '+row.place+'위':s.status==='simulation_error'?'결과 검토':'미완주 · 순위 제외';
    $('pilotPlace').textContent=rank;
    $('resultTitle').textContent=run().name+'  ·  '+rank;
    $('resultDesc').textContent=statusName(s.status)+' · '+s.gates_passed+'/'+s.gates_total+' 게이트 · '+(s.completed?'환산 기록 '+fmt(s.score_s):'비행시간 '+fmt(s.flight_time_s));
    $('result').hidden=false;$('next').textContent=current+1<runs.length?'다음 주자 입장 →':'다시 재생 ↻';paint();
  }
  function select(i){
    current=i;t=0;playing=false;ended=false;frameTime=null;eventKey='';const r=run(),l=log();
    $('bib').textContent='NO. '+String(i+1).padStart(2,'0');$('pilotName').textContent=r.name;$('pilotPlace').textContent='';
    $('sessionLabel').textContent=(r.synthetic?'예시 경기 · 모의 비행 데이터':'내 로그 · testbed 기록 재생')+'  /  '+(i+1)+'번째 주자';
    $('courseName').textContent=l.config.track.name+'  ·  '+l.config.track.gates.length+' gates';
    $('limitLabel').textContent='기준 '+l.scenario.load_limit_g+' g';
    $('ruleLoad').textContent='하중이 '+l.scenario.load_limit_g+' g를 초과한 양의 제곱을 시간에 따라 누적한 뒤 '+l.scenario.load_penalty_s+'를 곱해 비행시간에 더합니다.';
    $('ruleFailure').textContent='실패 시 '+l.scenario.failure_penalty_s+'초가 진단 기록에 더해집니다. 미완주는 순위에서 제외하고, 시뮬레이션 오류는 검토 대상으로 표시합니다.';
    $('result').hidden=true;board();paint();
  }
  function reset(){
    imported=false;runs=data.demoRuns.slice();completed=new Set(runs.slice(0,3).map(r=>r.id));
    rate=1;$('speed').value='1';select(Math.min(3,runs.length-1));
  }
  function resume(){playing=true;frameTime=null;paint();}
  function loop(now){
    if(playing){
      if(frameTime!==null)t=Math.min(total(),t+Math.max(0,now-frameTime)/1000*rate);
      if(t>=total())finish();else if(now-framePaint>=58){paint();framePaint=now;}
    }
    frameTime=now;requestAnimationFrame(loop);
  }
  $('play').onclick=()=>{if(ended)select(current);if(playing){playing=false;paint();}else resume();};
  $('next').onclick=()=>{select(current+1<runs.length?current+1:current);resume();};
  $('nearEnd').onclick=()=>{if(ended)select(current);t=Math.max(0,total()-3);resume();};
  $('reset').onclick=reset;
  $('scrub').oninput=()=>{const p=Number($('scrub').value)/1000;t=total()*p;frameTime=null;ended=false;$('result').hidden=true;$('pilotPlace').textContent='';if(p>=1)finish();else paint();};
  $('speed').onchange=()=>{rate=Number($('speed').value);frameTime=null;};
  function camera(mode){view=mode;$('view3d').setAttribute('aria-pressed',String(mode==='3d'));$('viewTop').setAttribute('aria-pressed',String(mode==='top'));paint();}
  $('view3d').onclick=()=>camera('3d');$('viewTop').onclick=()=>camera('top');
  function dialog(id){resumeOnClose=playing;playing=false;paint();$(id).showModal();}
  $('openRules').onclick=()=>dialog('rulesDialog');$('openImport').onclick=()=>{$('importError').textContent='';dialog('importDialog');};
  $('closeImport').onclick=()=>$('importDialog').close();
  for(const id of ['rulesDialog','importDialog'])$(id).addEventListener('close',()=>{if(resumeOnClose&&!ended)resume();resumeOnClose=false;});
  $('importGo').onclick=async()=>{
    const files=Array.from($('files').files);if(!files.length){$('importError').textContent='JSON 파일을 선택하세요.';return;}
    $('importGo').disabled=true;
    try{
      const batch=[];
      for(const f of files){
        try{if(f.size>32000000)throw Error('32 MB 이하의 로그를 선택하세요.');const l=data.normalizeLog(await f.text());batch.push({id:'file-'+(++importSeq),name:f.name.replace(/\.json$/i,'')||'참가자',log:l,synthetic:l.synthetic===true});}
        catch(e){throw Error(f.name+' — '+e.message);}
      }
      if(!imported){runs=[];completed=new Set();imported=true;}
      const first=runs.length;runs.push(...batch);resumeOnClose=false;$('importDialog').close();$('files').value='';select(first);
    }catch(e){$('importError').textContent=e.message;}
    finally{$('importGo').disabled=false;}
  };
  window.addEventListener('resize',()=>{if(runs.length)paint();});
  document.addEventListener('visibilitychange',()=>{if(document.hidden&&playing){playing=false;paint();}frameTime=null;});
  reset();requestAnimationFrame(loop);
})();
