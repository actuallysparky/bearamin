#!/usr/bin/env python3
import json
import pathlib
import re
import subprocess


PROJECT_ROOT = pathlib.Path(__file__).resolve().parents[1]
WORKER_HEADER = (
    PROJECT_ROOT / "firmware" / "BearaminSidecar" / "capture_worker_js.h"
)
SIDECAR = PROJECT_ROOT / "firmware/libraries/BearaminSidecar/src/BearaminSidecarApp.cpp"
MATRIXCORE = (
    PROJECT_ROOT / "firmware/libraries/BearaminMatrixCore/src/BearaminMatrixCoreApp.cpp"
)


def extract_worker() -> str:
    source = WORKER_HEADER.read_text()
    match = re.search(r'R"JS\((.*)\)JS";', source, re.S)
    assert match, "embedded capture worker raw string not found"
    return match.group(1)


def main() -> None:
    worker = extract_worker()
    sidecar = SIDECAR.read_text()
    matrixcore = MATRIXCORE.read_text()

    for required in (
        "const DB_NAME='bearamin-captures'",
        "createObjectStore('captures'",
        "createObjectStore('rawChunks'",
        "createObjectStore('artifactChunks'",
        "const RAW_CHUNK_BYTES=64*1024",
        "const FETCH_LIMIT=8192",
        "Date.now()-current.lastCommitAt>=1000",
        "await transactionDone(tx)",
        "ack=${ack>>>0}",
        "async function recover()",
        "active.status='interrupted'",
        "new CompressionStream('deflate-raw')",
        "while(captures.length>5)",
        "meta.exported&&meta.completedAt<cutoff",
    ):
        assert required in worker, f"missing worker contract: {required}"

    expected_files = (
        "manifest.json",
        "playback_frames.jsonl",
        "matrix.jsonl",
        "imu.jsonl",
        "control.jsonl",
        "diagnostics.jsonl",
        "gaps.jsonl",
    )
    for filename in expected_files:
        assert filename in worker, f"missing cooked artifact {filename}"

    raw_download = worker.index("requestDownload('raw'")
    cooking = worker.index("current.stage='cooking'", raw_download)
    json_download = worker.index("requestDownload('json'", cooking)
    assert raw_download < cooking < json_download, (
        "Stop must request the verified raw capture before cooking and JSON ZIP"
    )
    assert "live_controls_excluded:['knobs','mappings','dpt_mode','dpt_status','dpt_control_flags']" in worker
    assert "replayable_sensor_control:false" in worker
    assert "cells_mm:matrix?matrix.cells_mm:null" in worker
    assert "cell_statuses:matrix?matrix.statuses:null" in worker
    assert "record.type===5||record.type===11" in worker
    assert "pitch_norm:v2?u16(v,34):2048" in worker
    assert "roll_norm:v2?u16(v,36):2048" in worker
    assert "orientation_confidence:v2?v.getUint8(38):0" in worker
    assert "capture_schema:2" in worker

    for endpoint in (
        'server.on("/capture/start", HTTP_POST, handleCaptureStart)',
        'server.on("/capture/data", HTTP_GET, handleCaptureData)',
        'server.on("/capture/stop", HTTP_POST, handleCaptureStop)',
        'server.on("/capture/status", HTTP_GET, handleCaptureStatus)',
    ):
        assert endpoint in sidecar, f"missing bounded capture endpoint: {endpoint}"
    assert "kCapturePsramBytes = 1024 * 1024" in sidecar
    assert "captureFallbackBuffer[kCaptureFallbackBytes]" in sidecar
    assert "captureHttpBuffer[kCaptureHttpMaxBytes]" in sidecar
    assert "MatrixSerial.setRxBufferSize(kMatrixRxBufferBytes)" in sidecar
    assert "MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT" in sidecar
    assert "capture_response_allocation" not in sidecar
    assert "captureStore.AckThrough(ack)" in sidecar
    assert "rememberTelemetrySnapshot(snapshot, record.coreUs)" in sidecar
    state_branch = sidecar.split(
        "if (type == bearamin_sidecar::FRAME_STATE)", 1
    )[1].split(
        "} else if (type == bearamin_sidecar::FRAME_MATRIX_CELLS)", 1
    )[0]
    assert "rememberTelemetry" not in state_branch, (
        "legacy telemetry must not retain a second state-frame logging hot path"
    )
    telemetry_reset = sidecar.split("void handleTelemetryReset()", 1)[1].split(
        "void handleTelemetryStop()", 1
    )[0]
    assert "COMMAND_CAPTURE_START" in telemetry_reset
    assert "setEngineeringStream(true)" not in telemetry_reset

    assert "constexpr size_t kCaptureRingBytes = 16 * 1024" in matrixcore
    assert "SidecarSerial.availableForWrite()" in matrixcore
    assert "SidecarSerial.setTxBufferSize(kSidecarTxBufferBytes)" in matrixcore
    assert "kCaptureDrainMaxBytes" in matrixcore
    drain = matrixcore.split("void captureDrainToSidecar()", 1)[1].split(
        "uint16_t captureSensorFlags()", 1
    )[0]
    assert "captureRing.Peek(header, sizeof(header))" in drain
    assert "kCapturePriorityReserveBytes" in drain
    assert "drained + recordSize > kCaptureDrainMaxBytes" in drain
    assert "SidecarSerial.write(record, recordSize)" in drain
    assert "captureRing.Consume(recordSize)" in drain
    assert "ContiguousData()" not in drain
    loop = matrixcore.split("void loop()", 1)[1]
    assert loop.index("sendSidecarState(now)") < loop.index(
        "captureDrainToSidecar()"
    )
    assert loop.index("renderLeds(now)") < loop.index("captureDrainToSidecar()")
    dpt_send = matrixcore.split("const bearamin_bridge::Packet packet", 1)[1].split(
        "void rememberDptRxByte", 1
    )[0]
    assert dpt_send.index("DptSerial.write(frame, length)") < dpt_send.index(
        "captureProcessedSnapshot()"
    )
    assert "CONTROL_SIDECAR_COMMAND" in matrixcore
    assert "CONTROL_DPT_STATUS" in matrixcore
    assert "RECORD_CAPTURE_METADATA" in matrixcore
    assert "FLAG_PARTIAL" in matrixcore

    # Compile and exercise the worker's actual parser without starting recovery.
    node = f"""
global.self={{}};
global.postMessage=()=>{{}};
const source={json.dumps(worker)}.replace(/recover\\(\\);\\s*$/, '');
const api=new Function(source+'; return {{parseRecords,crc32,addFetched,decodeSnapshot,setCurrent:value=>current=value,getCurrent:()=>current}};')();
const payload=new Uint8Array([7,8,9]);
const bytes=new Uint8Array(22+payload.length+4);
bytes.set([66,67,65,80],0);
bytes[4]=1; bytes[5]=5;
const view=new DataView(bytes.buffer);
view.setUint16(6,0,true);
view.setUint16(8,payload.length,true);
view.setUint32(10,0xffffffff,true);
view.setUint32(14,123456,true);
view.setUint32(18,0,true);
bytes.set(payload,22);
view.setUint32(22+payload.length,api.crc32(bytes.subarray(0,22+payload.length)),true);
const parsed=api.parseRecords(bytes);
if(parsed.length!==1||parsed[0].sequence!==0xffffffff||parsed[0].coreUs!==123456)throw Error('worker parser roundtrip failed');
bytes[22]^=1;
let rejected=false;
try{{api.parseRecords(bytes)}}catch(error){{rejected=/CRC/.test(error.message)}}
if(!rejected)throw Error('worker parser accepted a corrupt record');
const snapshotV2=new Uint8Array(40),snapshotView=new DataView(snapshotV2.buffer);
snapshotView.setUint16(0,2,true);
snapshotView.setUint16(2,64,true);
snapshotView.setUint16(34,1234,true);
snapshotView.setUint16(36,3456,true);
snapshotView.setUint8(38,219);
const decodedSnapshot=api.decodeSnapshot({{payload:snapshotV2,coreUs:1000,sequence:7}},null);
if(decodedSnapshot.pitch_norm!==1234||decodedSnapshot.roll_norm!==3456||
   decodedSnapshot.orientation_confidence!==219||!decodedSnapshot.spank_gate)
  throw Error('worker snapshot v2 decode failed');
function encoded(type,sequence,payload){{
  const out=new Uint8Array(22+payload.length+4),v=new DataView(out.buffer);
  out.set([66,67,65,80],0);out[4]=1;out[5]=type;
  v.setUint16(8,payload.length,true);v.setUint32(10,sequence,true);
  out.set(payload,22);v.setUint32(22+payload.length,api.crc32(out.subarray(0,22+payload.length)),true);
  return out;
}}
const start=encoded(1,0,new Uint8Array(24));
const gapPayload=new Uint8Array(16),gapView=new DataView(gapPayload.buffer);
gapPayload[1]=1;gapView.setUint32(4,1,true);gapView.setUint32(8,3,true);gapView.setUint32(12,3,true);
const gap=encoded(8,3,gapPayload),joined=new Uint8Array(start.length+gap.length);
joined.set(start);joined.set(gap,start.length);
api.setCurrent({{haveSequence:false,expectedSequence:0,sequenceErrors:0,clientGaps:[],counts:{{}},pendingSegments:[],pendingBytes:0,rawBytes:0,recordCount:0,matrixDrops:0,sidecarGapDrops:0}});
api.addFetched(joined);
const accounting=api.getCurrent();
if(accounting.sequenceErrors!==0||accounting.clientGaps.length!==0||accounting.matrixDrops!==3)throw Error('explicit gap was double counted');
console.log('capture worker contract passed');
"""
    subprocess.run(["node", "-e", node], check=True, cwd=PROJECT_ROOT)


if __name__ == "__main__":
    main()
