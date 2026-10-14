'use strict';

const assert=require('node:assert/strict');
const fs=require('node:fs');
const path=require('node:path');
const test=require('node:test');
const vm=require('node:vm');

const source=fs.readFileSync(path.join(__dirname,'../app.js'),'utf8');
const renewal=source.slice(source.lastIndexOf('setInterval(()=>{if(device?.capabilities?.field&&!busy)request'));

function renew({field,busy}) {
  const calls=[];
  let timer;
  vm.runInNewContext(renewal,{
    device:field?{capabilities:{field:true}}:null,
    busy,
    request:(...args)=>{calls.push(args);return {catch:()=>{}};},
    setInterval:(callback,delay)=>{timer={callback,delay};},
  });
  assert.ok(timer,'Field Guide must register a preview-lease renewal timer.');
  timer.callback();
  return {calls,delay:timer.delay};
}

test('Field Guide renews only an active field-preview lease every minute',()=>{
  const active=renew({field:true,busy:false});
  assert.equal(active.delay,60000);
  assert.equal(active.calls.length,1);
  assert.equal(active.calls[0][0],'field.subscribe');
  assert.equal(active.calls[0][1].hz,5);
  assert.deepEqual(renew({field:false,busy:false}).calls,[]);
  assert.deepEqual(renew({field:true,busy:true}).calls,[]);
});
