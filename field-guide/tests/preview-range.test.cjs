const test=require('node:test'),assert=require('node:assert/strict'),fs=require('node:fs'),path=require('node:path'),vm=require('node:vm');
const source=fs.readFileSync(path.join(__dirname,'../app.js'),'utf8');
const helper=source.slice(source.indexOf('function previewStrength('),source.indexOf('function paint('));
const context=vm.createContext({});vm.runInContext(helper,context);
const strength=(mm,near,far)=>vm.runInContext(`previewStrength(${JSON.stringify(mm)},${JSON.stringify({near,far})})`,context);
test('preview uses the authoritative playable range and hides farther cells',()=>{
 assert.equal(strength(0,100,300),null);assert.equal(strength(null,100,300),null);
 assert.equal(strength(50,100,300),1);assert.equal(strength(100,100,300),1);
 assert.equal(strength(200,100,300),0.5);assert.equal(strength(300,100,300),0);
 assert.equal(strength(301,100,300),null);assert.equal(strength(500,100,300),null);
 assert.equal(strength(500,100,900),0.5);assert.equal(strength(300,300,300),null);
});
test('actual canvas paint follows frame range, then saved settings for old firmware',()=>{
 const paint=source.slice(source.indexOf('function paint('),source.indexOf('\nrender();paint();'));
 const colors=[], elements={};
 const ctx={fillStyle:'',fillRect(){},beginPath(){},roundRect(){},fill(){colors.push(this.fillStyle);}};
 const frame={mm:[100,200,300,301,500,...Array(59).fill(null)],x:64,y:64,z:64};
 const sandbox=vm.createContext({device:{capabilities:{field:true}},frame,saved:{near:100,far:300},draft:{near:100,far:900},paused:false,lastFrame:Date.now(),$:(id)=>id==='matrix'?{getContext:()=>ctx}:(elements[id]??={style:{}})});
 vm.runInContext(helper+paint+';paint();',sandbox);
 assert.equal(colors.length,64);assert.equal(colors[0],'rgb(44,208,213)');
 assert.equal(colors[1],'rgb(32,131,126)');assert.equal(colors[2],'rgb(20,53,38)');
 assert.equal(colors[3],'#0b1c14');assert.equal(colors[4],'#0b1c14');
 // The device range wins over saved/unsaved client state on new firmware.
 frame.near=100;frame.far=900;colors.length=0;vm.runInContext('paint()',sandbox);
 assert.equal(colors[4],'rgb(32,131,126)');
});
