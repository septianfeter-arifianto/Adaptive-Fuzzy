let V=1, f=50, phase=0, t=0;
let gain=1.0, mode="FAI", faultOn=false;
let faultType = "C1";
let faultStart = 0;
let faultDuration = 0.25;
let faultDropdown;
let loadMW = 4;
let histN = 350;   
let capSliders = [];
let lastClick = 0;
let sliderX = 750;
let sliderY = 150;
let sliderW = 250;



// PID states
let e_int=0, e_prev=0;
let Kp=6, Ki=1.5, Kd=0.05;

// history for scope
let histV=[], histF=[];

let DGs = [
  {name:"PV", x:120,y:140, P:0.6,Q:0.1, V:1,f:50, histV:[], histF:[], Kp:0.8, Kq:0.6, cap:1.0, histP:[]},
  {name:"Wind", x:120,y:220, P:0.7,Q:0.15,V:1,f:50, histV:[], histF:[], Kp:0.6, Kq:0.5, cap:1.0, histP:[]},
  {name:"FuelCell", x:120,y:300, P:0.5,Q:0.05,V:1,f:50, histV:[], histF:[], Kp:1.0, Kq:0.7, cap:1.0, histP:[]},
  {name:"MicroAlt", x:120,y:380, P:0.8,Q:0.2,V:1,f:50, histV:[], histF:[], Kp:0.5, Kq:0.4, cap:1.0, histP:[]},
];

let P_load = 2.5;
let Q_load = 0.8;

function setup(){
faultDropdown = createSelect();
faultDropdown.position(360, 55);
faultDropdown.option("C0=1");
faultDropdown.option("C1=0.7");
faultDropdown.option("C2=0.5");
faultDropdown.option("C3=0.0");
faultDropdown.selected("C1");
  createCanvas(1200,650);
  textFont('monospace');

  //  slider kapasitas tiap DG
  DGs.forEach((dg,i)=>{

    let s = createSlider(0.25, 1.5, 0.75, 0.1);

    // posisi di Atas DG
    s.position(dg.x -5, dg.y -18);

    s.style('width','95px');

    capSliders.push(s);
  });
}

function draw(){
  background(245);

  DGs.forEach((dg,i)=>{
    dg.cap = capSliders[i].value();
  });

  P_load = loadMW;
  t += 0.03;

  V += random(-0.002,0.002);
  f += random(-0.01,0.01);

if(mouseIsPressed &&
   mouseX > sliderX && mouseX < sliderX + sliderW &&
   mouseY > sliderY - 10 && mouseY < sliderY + 10){

  let mx = constrain(mouseX, sliderX, sliderX + sliderW);
  loadMW = map(mx, sliderX, sliderX + sliderW, 1, 4);
}

  // ===== 🔥 TARUH DI SINI =====
  let fault = 1.0;

  let selected = faultDropdown.value();

  if(selected === "C0=1"){
    fault = 1;
  }
  if(selected === "C1=0.7"){
    fault = 0.7;
  }
  else if(selected === "C2=0.5"){
    fault = 0.5;
  }
  else if(selected === "C3=0.0"){
    fault = 0.0;
  }

  // ===== CONTROL =====
  let e = 1 - V;
  let u = (mode==="FAI") ? FAI(e) : PID(e);

  // ===== LOAD SHARING =====
let invKsum = DGs.reduce((s,d)=>s + 1/d.Kp,0);

DGs.forEach(dg => {

  // ===== ACTIVE POWER (P) =====
  let share = (1/dg.Kp) / invKsum;
  dg.P = share * P_load;

  //  LIMIT KAPASITAS
  dg.P = constrain(dg.P, 0, dg.cap);

  // ===== REACTIVE POWER (Q) =====
  let V0 = 1.0;

  // Q-V droop
  let Qref = (V0 - dg.V) / dg.Kq;

  // smoothing
  dg.Q += 0.05*(Qref - dg.Q);

  // limit Q
  dg.Q = constrain(dg.Q, -0.5, 0.5);
});

DGs.forEach(dg=>{
  pushHist(dg.histP, dg.P);
});

// =====NORMALISASI TOTAL POWER =====
let Ptotal = DGs.reduce((s,d)=>s+d.P,0);

if(Ptotal > 0){

  let scale = P_load / Ptotal;

  DGs.forEach(dg=>{
    dg.P *= scale;

    // tetap tidak boleh melebihi kapasitas
    dg.P = constrain(dg.P, 0, dg.cap);
  });
}

  // simple plant (aggregate)
  let Psum = DGs.reduce((s,d)=>s+d.P,0);
  let Qsum = DGs.reduce((s,d)=>s+d.Q,0);

  let loadImpact = -0.1 * (loadMW - 2.5);
  let dV = -2.5*(V - fault) + 0.12*(Psum-2.6) - 0.08*(Qsum-0.5) + u + loadImpact;
  V += dV * 0.05;

  // freq droop
  let f_ref = 50 + (1 - V)*2.0 - 0.3*(loadMW - 2.5);
  f += 0.1*(f_ref - f);
  phase += TWO_PI*f*0.0005;

  // learning (for FAI)
  if(mode==="FAI"){
    let reward = -abs(e);
    gain += 0.01*reward;
    gain = constrain(gain, 0.5, 3);
  }

  // update scope
  pushHist(histV, V);
  pushHist(histF, f);

  // ===== MULTI BUS UPDATE =====
DGs.forEach(dg => {
  dg.V += 0.1*(V - dg.V) + random(-0.001,0.001);
  dg.f += 0.1*(f - dg.f) + random(-0.01,0.01);

  pushHist(dg.histV, dg.V);
  pushHist(dg.histF, dg.f);
});
  
  drawUI(fault, f_ref);
  drawNetwork(Psum, Qsum);
  drawmultiScope();
  drawLoadSharingChart();
  drawScopeP(600, 500, 550, 120);
}

// ===================== CONTROLLERS =====================
function FAI(e){
  let NB=tri(e,-1,-1,-0.5), NS=tri(e,-1,-0.5,0),
      Z =tri(e,-0.5,0,0.5),  PS=tri(e,0,0.5,1),
      PB=tri(e,0.5,1,1);

  let w=[1.5,1.2,1,1.2,1.5];
  let out = NB*w[0]*(-10)+NS*w[1]*(-5)+Z*w[2]*0+PS*w[3]*5+PB*w[4]*10;
  let sum = NB*w[0]+NS*w[1]+Z*w[2]+PS*w[3]+PB*w[4];
  if(sum===0) return 0;
  return gain*(out/sum);
}

function PID(e){
  e_int += e;
  let de = e - e_prev;
  e_prev = e;
  return Kp*e + Ki*e_int + Kd*de;
}

function tri(x,a,b,c){
  return max(min((x-a)/(b-a),(c-x)/(c-b)),0);
}

// ===================== DRAW =====================
function drawUI(fault, f_ref){

  // ===== TITLE =====
  fill(0);
  textSize(16);
  textAlign(CENTER);
  text("Adaptive Fuzzy Attention Inference to Control a Microgrid Under Extreme Fault on Grid Bus", width/2, 25);
  if(loadMW > 6){
  fill(255,0,0);
  textSize(14);
  text("⚠ OVERLOAD", sliderX + 80, sliderY - 20);
}

  // ===== BUTTON ROW =====
  textAlign(LEFT);
  drawButton(40,50,140,30,"MODE: "+mode);
  drawButton(200,50,140,30,"FAULT: "+faultDropdown.value());


  // ===== STATUS ROW (TURUNKAN KE BAWAH) =====
  fill(0);
  textSize(12);

  let yStatus = 100; // ⬅️ ini kunci (turunin)

  text(`Voltage: ${V.toFixed(3)} pu`, 40, yStatus);
  text(`Frequency: ${f.toFixed(2)} Hz`, 220, yStatus);
  text(`f_ref: ${f_ref.toFixed(2)} Hz`, 420, yStatus);
  text(`Gain: ${gain.toFixed(2)}`, 600, yStatus);

  // ===== FAULT WARNING =====
  if(fault < 1){
    fill(200,0,0);
    textSize(16);
    text("GRID FAULT", width/2 - 60, 130);
  }
// ===== 👉 TAMBAHKAN SLIDER DI SINI =====

  drawLoadSlider(sliderX, sliderY, sliderW);

  fill(0);
  textSize(12);
  text("GRID Load (MW)", sliderX, sliderY - 10);

  text(loadMW.toFixed(2) + " MW", sliderX + sliderW + 20, sliderY + 5);
}

function drawNetwork(Psum, Qsum){
  let PCC = {x:700, y:260};
  let GRID= {x:920, y:260};
  let STAT= {x:700, y:420};

  // ===== 1. DRAW DG + INV + LINES =====
  DGs.forEach((dg,i)=>{
    drawDG(dg.x, dg.y, dg.name, dg);
    let invX = dg.x + BLOCK_W + 20;   // jarak konsisten
let invY = dg.y + (BLOCK_H/2 - 20);

fill(70,160,220);   // biru rapi
stroke(120);
rect(invX, invY, 80, 50, 6);

fill(0);
noStroke();
textAlign(CENTER, CENTER);
textSize(10);
text("INV", invX + 40, invY + 12);
text(`P:${dg.P.toFixed(2)}`, invX + 40, invY + 24);
text(`Q:${dg.Q.toFixed(2)}`, invX + 40, invY + 36);


    
    drawPQLine(dg.x+220, dg.y+30, PCC.x, PCC.y, dg.P, dg.Q);
  });

  // ===== 2. DRAW ALL CONNECTION LINES =====
  drawPQLine(PCC.x, PCC.y, GRID.x-40, GRID.y, Psum, Qsum);

  let Qst = 0.3*(1 - V);
  drawPQLine(STAT.x, STAT.y-30, PCC.x, PCC.y, 0, Qst);

  // ===== 3. DRAW NODES TERAKHIR (INI KUNCI 🔥) =====
  drawBlock(PCC.x-60, PCC.y-20, "PCC");
fill(0);
textAlign(CENTER);
textSize(10);
text(`P:${Psum.toFixed(2)}`, PCC.x, PCC.y + 20);
text(`Q:${Qsum.toFixed(2)}`, PCC.x, PCC.y + 35);    
  drawBlock(GRID.x-60, GRID.y-20, "GRID");
  fill(0);
textAlign(CENTER);
textSize(10);
text(`P:${Psum.toFixed(2)}`, GRID.x, GRID.y + 20);
text(`Q:${Qsum.toFixed(2)}`, GRID.x, GRID.y + 35);
  drawBlock(STAT.x-60, STAT.y-20, "STATCOM");
  
// tampilkan Q
fill(0);
textAlign(CENTER);
textSize(10);

text(`Q:${Qst.toFixed(2)} MVAr`, STAT.x, STAT.y + 20);
}

function drawDG(x,y,label,dg){

  push();

  // ===== BOX DG =====
  stroke(120);          // semua sama
  strokeWeight(1);      // ⬅️ ini kunci (hapus tebal PV)
  fill(230);
  rect(x,y,BLOCK_W,BLOCK_H,8);

  // ===== TEXT =====
  fill(0);
  noStroke();
  textSize(10);
  textAlign(LEFT, TOP);

  text(label, x+8, y+6);

  text(`V:${dg.V.toFixed(2)}`, x+8, y+20);
  text(`f:${dg.f.toFixed(1)}`, x+65, y+20);

  text(`P:${dg.P.toFixed(2)}`, x+8, y+35);
  text(`Q:${dg.Q.toFixed(2)}`, x+65, y+35);

  // indikator fase
  let ph = phase % TWO_PI;
  fill(0,180,255);
  arc(x+100, y-8, 18, 18, 0, ph);

  pop();
}

function pushHist(arr, val){
  arr.push(val);        // tambah data baru

  // jaga agar tidak terlalu panjang
  if(arr.length > histN){
    arr.shift();        // hapus data lama
  }
}

function drawBlock(x,y,label){
  fill(220);
  rect(x,y,BLOCK_W,BLOCK_H,8);

  fill(0);
  textSize(11);
  textAlign(CENTER, CENTER);
  text(label, x + BLOCK_W/2, y + BLOCK_H/2);
}

// P (green) + Q (blue)
function drawPQLine(x1,y1,x2,y2,P,Q){
  // base line
  stroke(120);
  line(x1,y1,x2,y2);

  // P flow (green)
  stroke(0,180,0);
  strokeWeight(1 + 3*abs(P));
  line(x1,y1,x2,y2);

  // moving dot for P
  let s = (sin(frameCount*0.12)+1)/2;
  let xp = lerp(x1,x2,s), yp = lerp(y1,y2,s);
  noStroke(); fill(0,180,0);
  ellipse(xp,yp,6);

  // Q flow (blue, slightly offset)
  stroke(0,120,255);
  strokeWeight(1 + 3*abs(Q));
  line(x1,y1+4,x2,y2+4);

  let s2 = (cos(frameCount*0.1)+1)/2;
  let xq = lerp(x1,x2,s2), yq = lerp(y1+4,y2+4,s2);
  noStroke(); fill(0,120,255);
  ellipse(xq,yq,6);

  strokeWeight(1);
  noStroke();
}

function drawButton(x,y,w,h,label){
  fill(220);
  rect(x,y,w,h,6);
  fill(0);
  textSize(12);
  text(label, x+10, y+20);
}


function drawLoadSharingChart(){

  let x = 990;
  let y = 220;
  let w = 200;
  let h = 100;

  // frame
  stroke(0);
  noFill();
  rect(x,y,w,h);

  fill(0);
  noStroke();
  textSize(12);
  textAlign(CENTER);
  text("LOAD per DG", x + w/2, y - 10);

  let barW = w / DGs.length;

  let totalP = DGs.reduce((s,d)=>s + d.P, 0);
  
  DGs.forEach((dg,i)=>{

    let barHeight = map(dg.P, 0, 1, 0, h-40);

    let bx = x + i * barW + 10;
    let by = y + h - barHeight;

    let colors = [
      [0,180,0],
      [0,120,255],
      [255,100,0],
      [200,0,200]
    ];
    // ===== TOTAL LOAD TEXT =====
fill(0);
textSize(13);
textAlign(CENTER);

text("Total Load: " + totalP.toFixed(2) + " MW", x + w/2, y + h + 35);

    fill(colors[i][0], colors[i][1], colors[i][2]);
    rect(bx, by, barW - 20, barHeight);

    // label
    fill(0);
    textSize(10);
    text(dg.name, bx + (barW-20)/2, y + h + 15);

    // nilai
    text(dg.P.toFixed(2), bx + (barW-20)/2, by - 5);
  });
}

// ===================== SCOPE =====================
function drawmultiScope(){
  let x=60, y=500, w=500, h=120;

  // frame
  stroke(0); noFill();
  rect(x,y,w,h);

  noStroke();
  fill(0);
  textSize(11);
  text("Graph (V, f)", x+40, y-10);

  
  // ===== GRID SCALE =====
  stroke(200);
  for(let i=0;i<=4;i++){
    let yy = map(i,0,4,y+h,y);
    line(x,yy,x+w,yy);
  }

  noStroke();
  fill(0);
  text("1.2", x-30, y);
  text("1.0", x-30, y+h/2);
  text("0.5", x-30, y+h);

  // ===== PLOT V =====
  stroke(0,180,0); noFill();
  beginShape();
  for(let i=0;i<histV.length;i++){
    let xi = map(i, 0, histN-1, x, x+w);
    let yi = map(histV[i], 0.5, 1.2, y+h, y);
    vertex(xi, yi);
  }
  endShape();

  // ===== PLOT f =====
  stroke(0,120,255); noFill();
  beginShape();
  for(let i=0;i<histF.length;i++){
    let xi = map(i, 0, histN-1, x, x+w);
    let yi = map(histF[i], 49, 51, y+h, y);
    vertex(xi, yi);
  }
  endShape();

  // ===== NILAI TERKINI =====
  fill(0);
  textSize(12);
  text(`V = ${V.toFixed(3)} pu`, x+60, y+15);
  text(`f = ${f.toFixed(2)} Hz`, x+200, y+15);
}

function pushHist(arr, val){
  arr.push(val);
  if(arr.length>histN) arr.shift();
}
function mouseClicked(){

  // MODE BUTTON
  if(hit(40,50,140,30)){
    mode = (mode==="FAI") ? "PID" : "FAI";
  }

  // FAULT BUTTON
  if(hit(200,50,140,30)){
    if(faultOn){
      faultStart = 0;
    }
  }

  // ===== SLIDER =====
  if(mouseX > sliderX && mouseX < sliderX + sliderW &&
     mouseY > sliderY - 10 && mouseY < sliderY + 10){
    dragging = true;
  }
}


function hit(x,y,w,h){
  return mouseX>x && mouseX<x+w && mouseY>y && mouseY<y+h;
}

DGs.forEach(dg => {

  // P-f droop
  let f0 = 50;
  dg.f = f0 - dg.Kp * (dg.P - 0.6);

  // Q-V droop
  let V0 = 1.0;
  dg.V = V0 - dg.Kq * (dg.Q - 0.1);

  // sinkronisasi ke PCC
  dg.V += 0.08*(V - dg.V);
  dg.f += 0.08*(f - dg.f);

  // simpan history
  pushHist(dg.histV, dg.V);
  pushHist(dg.histF, dg.f);
});

let BLOCK_W = 120;
let BLOCK_H = 60;

// DG
stroke(0,100,255);

// INV
stroke(150);

function drawLoadSlider(x,y,w){

  // garis slider
  stroke(180);
  strokeWeight(4);
  line(x, y, x+w, y);

  // posisi knob
  let pos = map(loadMW, 1, 4, x, x+w);

  // knob
  fill(0,120,255);
  stroke(0);
  ellipse(pos, y, 14, 14);

  // tick 1–4
  stroke(0);
  for(let i=1;i<=4;i++){
    let tx = map(i,1,4,x,x+w);
    line(tx, y-5, tx, y+5);

    noStroke();
    fill(0);
    textAlign(CENTER);
    text(i, tx, y+20);
  }
}

function drawArrow(x1,y1,x2,y2,label,color){

  stroke(color);
  strokeWeight(1.5);
  line(x1,y1,x2,y2);

  // kepala panah
  let angle = atan2(y2-y1, x2-x1);
  let size = 6;

  push();
  translate(x2,y2);
  rotate(angle);
  fill(color);
  triangle(0,0, -size,-size/2, -size,size/2);
  pop();

  // label
  noStroke();
  fill(color);
  textSize(11);
  text(label, x2 + 5, y2 - 5);
}

function drawScopeP(x,y,w,h){

  // ===== FRAME =====
  stroke(0);
  noFill();
  rect(x,y,w,h);
   // ===== GRID & SCALE =====
  stroke(200);

  let maxP = max(DGs.map(d=>d.cap));

  for(let i=0;i<=4;i++){
    let val = i * (maxP/4);
    let yy = map(val, 0, maxP, y+h, y);

    line(x, yy, x+w, yy);

    noStroke();
    fill(0);
    textSize(10);
    textAlign(RIGHT);
    text(val.toFixed(2), x - 5, yy + 3);
  }

  // ===== JUDUL =====
  fill(0);
  noStroke();
  textSize(12);
  text("Beban (P per DG)", x+60, y-10);

  // ===== WARNA TIAP DG =====
  let colors = [
    color(0,180,0),   // PV
    color(0,120,255), // Wind
    color(255,120,0), // FuelCell
    color(200,0,200)  // MicroAlt
  ];

  // ===== GAMBAR GRAFIK =====
  DGs.forEach((dg,i)=>{

    stroke(colors[i]);
    noFill();
    beginShape();

    for(let k=0;k<dg.histP.length;k++){

      let xx = map(k, 0, histN, x, x+w);

      // skala 0 – 1.5 MW
      let maxP = max(DGs.map(d=>d.cap));
      let yy = map(dg.histP[k], 0, maxP, y+h, y);

      vertex(xx,yy);
    }

    endShape();

    // ===== LABEL NILAI =====
    fill(colors[i]);
    noStroke();
    text(
      dg.name + ": " + dg.P.toFixed(2) + " MW",
      x+w-250,
      y-60 + i*15
    );
  });
}