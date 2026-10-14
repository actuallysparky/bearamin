const test=require('node:test'),assert=require('node:assert/strict');
const P=require('../protocol.js');
test('bounded SysEx roundtrip includes UTF8 and binary payload bytes',()=>{
  const v={op:'update.chunk',request:7,uid:'bear-ü',data:{bytes:Array.from({length:128},(_,i)=>i*2)}};
  const bytes=P.encode(v);assert.ok(Array.from(bytes.slice(1,-1)).every(v=>v<128));assert.deepEqual(P.decode(bytes),v);
  assert.equal(P.decode(new Uint8Array([0xf0,0x01,0xf7])),null);assert.throws(()=>P.encode({s:'x'.repeat(4097)}));
});
test('update decisions never offer a downgrade or mismatched reinstall',()=>{
  const r={build:12,imageSha256:'abc'};
  assert.equal(P.updateState({build:11},r).label,'Update');assert.equal(P.updateState({build:11},r).enabled,true);
  assert.equal(P.updateState({build:12,digest:'abc'},r).label,'Re-install');
  assert.equal(P.updateState({build:12,digest:'different'},r).enabled,false);
  assert.equal(P.updateState({build:13},r).enabled,false);assert.equal(P.updateState(null,r).enabled,false);
  assert.equal(P.updateState({build:11},null).enabled,false);assert.equal(P.updateState({build:'11'},r).enabled,false);
});
test('identity requires a supported device, schema and monotonic build',()=>{
  const identity={protocol:1,model:'virtual',uid:'v1',hardware:'2040',version:'1.0.0',build:1,capabilities:{update:'uf2'}};
  assert.equal(P.validIdentity(identity),true);assert.equal(P.validIdentity({...identity,model:'sidecar'}),false);
  assert.equal(P.validIdentity({...identity,uid:''}),false);assert.equal(P.validIdentity({...identity,build:-1}),false);
});
