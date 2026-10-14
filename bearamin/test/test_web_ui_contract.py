#!/usr/bin/env python3
import pathlib
import re
import subprocess


PROJECT_ROOT = pathlib.Path(__file__).resolve().parents[1]
SIDECAR = PROJECT_ROOT / "firmware/libraries/BearaminSidecar/src/BearaminSidecarApp.cpp"


def extract_html():
    source = SIDECAR.read_text()
    matches = re.findall(
        r'const char INDEX_HTML\[\] PROGMEM = R"HTML\((.*?)\)HTML";', source, re.S
    )
    assert matches, "production INDEX_HTML raw string not found"
    return matches[-1]


def main():
    source = SIDECAR.read_text()
    active_source = source.split("const char INDEX_HTML", 1)[1]
    html = extract_html()
    script_match = re.search(r"<script>(.*)</script>", html, re.S)
    assert script_match, "embedded script not found"
    script = script_match.group(1)

    # Product decisions: the exact Soundfield mark is served and CV8 is the
    # only authority for DPT mode, including when a preset is restored.
    assert "kSoundfieldLogoSvg" in source
    assert "server.send_P(200, \"image/svg+xml\", kSoundfieldLogoSvg)" in source
    assert "COMMAND_SET_DPT_MODE" not in active_source
    assert "requestedDptMode" not in active_source
    for removed in (
        "res_particle_",
        "res_feedback_tap",
        "res_feedback_danger",
        "Particle dust",
        "bear-sized bucket of feedback",
        "map_glitch_",
        "name:'Glitch'",
    ):
        assert removed not in active_source, f"retired feature remains active: {removed}"
    assert "version:10" in active_source
    assert "res_frequency_quantization" in active_source
    assert "['off','Off'],['chromatic','Chromatic'],['major','Major'],['pentatonic_major','Pentatonic Major']" in active_source
    for contract in (
        "Sensor pipeline",
        "8×8 source · 15 Hz sensing · 30 Hz output",
        "matrix_source_width",
        "matrix_source_height",
        "matrix_source_hz",
        "matrix_output_hz",
        "matrix_effective_hz",
        "synthetic_gate",
    ):
        assert contract in active_source, f"missing fixed matrix UI contract: {contract}"
    for removed in (
        '/matrix/mode',
        "COMMAND_SET_MATRIX_MODE",
        'name="matrix_mode"',
        "matrix_requested_mode",
        "matrix_fast_mode",
        "4x4",
        "4×4",
        "60 Hz",
    ):
        assert removed not in active_source, f"obsolete matrix UI remains: {removed}"
    assert "CV6 chooses the displayed root and key" in active_source
    assert "frequencyRootNoteName" in active_source
    for value, label in (("pitch", "Pitch"), ("roll", "Roll"), ("spank", "Spank")):
        option = f"['{value}','{label}']"
        assert active_source.count(option) == 1, (
            f"{label} must occur once in the executable shared source list"
        )
    assert 'role="list" aria-label="Read-only DPT mode positions"' in html
    assert "Selected by the physical DPT CV8 knob" in html
    assert "data-mode=" not in script
    assert "sendSettings('mode=" not in script
    assert "const mappingConfigs=" in script
    assert "settingsKeys=['dpt_mode'" not in script
    assert "version:10" in script
    assert "case 0:\n    case 1:\n    case 2: return kGestureConfigRingsResonator;" in active_source
    assert "Preset versions 1-3 used numeric keys" not in active_source
    assert "map_%u_%u_axis" not in active_source
    assert "announce_modes" in script

    # The approved UX is present in the executable page rather than remaining
    # only in the design proposal.
    for text in (
        "★ default · Manage",
        "Save preset",
        "Overwrite default",
        "Reload default",
        "Loads on startup",
        "Gesture gate",
        "Arm OTA for 90 seconds",
        "Open live diagnostics",
        "Download diagnostics",
        "Choose JSON",
        "Capture",
        "Announce modes",
        "Leave this off for seamless switching",
    ):
        assert text in html, f"missing approved UI copy: {text}"
    assert "/presets.json" in script
    assert "/presets/save?name=" in script
    assert "/presets/load?name=" in script
    assert "dpt_control_tx_quiet" in script
    assert "Preset storage active · motion control remains live" in script
    assert "name==='default'?-1" in script
    assert "savePreset('default')" in script
    assert "Save to SD card" not in html
    assert "NVS" not in html
    assert "DPT SD · default loads at boot" not in html
    assert "ota_confirm" in script
    assert "engineering/stream?enabled=" in script
    assert "new Worker('/capture-worker.js')" in script
    assert "/telemetry/reset" not in script
    assert "showSaveFilePicker" not in script
    assert "prompt(" not in script
    assert "crypto.getRandomValues" in script
    assert "bearamin-capture-${stamp}" in script
    assert "dropped records" in script
    assert 'id="capture_compact"' in html
    assert 'id="capture_system"' in html
    instrument_html = html.split('<section id="instrument_view">', 1)[1].split(
        '<section id="system_view"', 1
    )[0]
    assert "capture_metrics" not in instrument_html
    assert "capture_library" not in instrument_html
    assert "ring" not in instrument_html.lower()
    assert "[hidden]{display:none!important}" in html
    assert "@media(max-width:900px)" in html
    expected_modes = (
        "name:'Modal'",
        "name:'Sympathetic Strings'",
        "name:'Karplus-Strong'",
        "name:'Bearamin FM drone'",
        "name:'Bearamin CV'",
    )
    positions = [script.index(mode) for mode in expected_modes]
    assert positions == sorted(positions), "mode labels are not in the requested order"
    assert script.count("mapKey:'rings_resonator'") == 3
    assert "grid-template-columns:repeat(5,minmax(0,1fr))" in html
    assert "All LFO" not in active_source
    assert "all_lfo" not in active_source
    assert "cvOut:['X axis|unipolar','Y axis|unipolar','Z axis|unipolar','X axis|bipolar','Y axis|bipolar','Z axis|bipolar']" in script
    assert '\\"wifi_ssid\\"' in active_source
    assert "WiFi.SSID()" in active_source
    assert "Gesture response (slew)" in script
    assert "Bearamin CV amount" in script
    assert "lane:last-child .lane-value" in script
    assert "latestDevice.wifi_ssid" not in script
    assert "wifiReady ? WiFi.SSID() : networkModeText()" in active_source

    harness = r'''
const elements = {};
const documentEvents = {};
function element(id) {
  if (!elements[id]) elements[id] = {
    id, innerHTML:'', textContent:'', className:'', hidden:false, disabled:false,
    value:'', checked:false, dataset:{}, style:{}, classList:{toggle(){},add(){},remove(){}},
    appendChild(){}, before(){}, click(){}, remove(){}, focus(){}, blur(){}, contains(){return false},
    querySelectorAll(){return []}, oninput:null, onchange:null, onclick:null
  };
  return elements[id];
}
global.document = {
  activeElement:null, body:{appendChild(){}},
  createElement(){return {textContent:'',href:'',download:'',click(){},remove(){},get innerHTML(){return String(this.textContent).replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/>/g,'&gt;')}}},
  getElementById:element, querySelector(selector){return selector.startsWith('#resonator_reverb_detail')?null:element('response_h2')}, querySelectorAll(){return []}, addEventListener(type, handler){documentEvents[type]=handler}
};
global.window = {addEventListener(){}};
global.navigator = {sendBeacon(){}};
global.URL = {createObjectURL(){return 'blob:test'}, revokeObjectURL(){}};
global.Blob = function(){};
global.confirm = () => true;
global.Worker = class {
  constructor(url) { this.url=url; this.messages=[]; global.captureWorkerMock=this; }
  postMessage(message) { this.messages.push(message); }
};
global.fetch = async () => ({json:async()=>({ok:true,presets:[]})});
global.setInterval = () => 0;
global.setTimeout = () => 0;
'''
    assertions = r'''
(async () => {
function assert(condition, message) { if (!condition) throw new Error(message); }
const params = settingsToParams({
  dpt_mode: 3, announce_modes: false, fm_drone_ranger: true,
  map_bearamin_fm_0_axis: 'x_bipolar', map_bearamin_fm_0_depth: -37,
  map_bearamin_fm_1_axis: 'z_bipolar', map_bearamin_fm_1_depth: 83,
  cv_slew_step: 28
});
assert(!/(^|&)mode=/.test(params), 'a preset import must never request a DPT mode');
assert(!params.includes('announce_modes='), 'announcement setting must not persist in v10 presets');
assert(params.includes('map_bearamin_fm_0_axis=x_bipolar'), 'stable FM mapping axis missing');
assert(params.includes('map_bearamin_fm_0_depth=-37'), 'fine signed FM depth missing');
assert(params.includes('map_bearamin_fm_1_depth=83'), 'positive fine FM depth missing');
assert(params.includes('map_bearamin_fm_1_axis=z_bipolar'), 'valid FM axis missing');
const experimentalParams = settingsToParams({
  map_bearamin_fm_0_axis:'pitch',
  map_rings_resonator_1_axis:'roll'
});
assert(experimentalParams.includes('map_bearamin_fm_0_axis=pitch'), 'Pitch preset round-trip missing');
assert(experimentalParams.includes('map_rings_resonator_1_axis=roll'), 'Roll preset round-trip missing');
assert(params.includes('cv_slew=28'), 'CV slew missing');
assert(params.includes('res_feedback_amount=0'), 'feedback default missing');
assert(params.includes('res_drone_waveform=sine'), 'drone default missing');
assert(params.includes('res_reverb_time=67'), 'reverb default missing');
assert(params.includes('res_frequency_quantization=off'), 'quantization default missing');
assert(!params.includes('res_particle_'), 'particle settings must not be serialized');
assert(!params.includes('res_feedback_tap'), 'feedback tap must not be serialized');
assert(!params.includes('res_feedback_danger'), 'danger feedback must not be serialized');

latestDevice = {dpt_mode: 2, firmware_version:'test', announce_modes:false, fm_drone_ranger:false, map_bearamin_fm_0_axis:'x_bipolar', map_bearamin_fm_0_depth:37, res_frequency_quantization:'major'};
const snap = snapshotSettings();
assert(!Object.prototype.hasOwnProperty.call(snap.settings, 'dpt_mode'), 'snapshots must not retain mode');
assert(snap.version === 10, 'current snapshot version missing');
assert(snap.settings.res_frequency_quantization === 'major', 'quantization setting missing from snapshot');
assert(!Object.prototype.hasOwnProperty.call(snap.settings, 'announce_modes'), 'announcement setting must not be snapshotted');
assert(!Object.keys(snap.settings).some(key=>key.startsWith('res_particle_')), 'particle settings must not be snapshotted');
assert(Object.prototype.hasOwnProperty.call(snap.settings, 'map_bearamin_fm_0_axis'), 'stable FM mapping identifier missing');
assert(!Object.prototype.hasOwnProperty.call(snap.settings, 'map_0_0_axis'), 'new snapshots must not persist dial indices');

renderAnnouncementSetting({announce_modes:false,announce_modes_applied:false});
assert(elements.announce_modes.checked === false, 'announcement checkbox must default off');
assert(elements.announce_modes_state.textContent === 'Applied on DPT', 'announcement apply state missing');

const ringsStatus={dpt_mode:1,dpt_knobs:[0,64,128,192,255,32,0,0],cv_slew_step:12,map_rings_resonator_2_axis:'y_unipolar',map_rings_resonator_2_depth:-73,res_feedback_amount:24,res_feedback_source:'z_unipolar',res_feedback_depth:18,res_drone_enabled:true,res_drone_waveform:'saw',res_drone_level:20,res_drone_source:'x_bipolar',res_drone_depth:10,res_reverb_time:67,res_reverb_time_source:'cv5_wet',res_reverb_time_depth:15,res_reverb_diffusion:63,res_reverb_diffusion_source:'none',res_reverb_diffusion_depth:0,res_reverb_lp:60,res_reverb_lp_source:'y_unipolar',res_reverb_lp_depth:-20,resonator_controls_applied:true};
renderPanel(ringsStatus);
assert(elements.mode_tabs.innerHTML.includes('aria-current="true"'), 'current physical mode should be announced');
assert(!elements.mode_tabs.innerHTML.includes('data-mode='), 'mode display must not be actionable');
assert(elements.cv_in.innerHTML.includes('map_rings_resonator_2_axis'), 'shared Rings axis control missing from CV lane');
assert(elements.cv_in.innerHTML.includes('map_rings_resonator_2_depth'), 'shared Rings depth attenuverter missing from CV lane');
assert(elements.cv_in.innerHTML.includes('Modulation depth attenuverter'), 'main modulation depth column heading missing');
assert(elements.cv_in.innerHTML.includes('-73%'), 'fine depth readout missing');
assert(elements.cv_in.innerHTML.includes('--depth-position:13.5%'), 'depth readout should follow the slider thumb');
assert(elements.cv_in.innerHTML.includes('--angle:0deg'), 'zero knob should render empty sweep');
assert(elements.cv_in.innerHTML.includes('--angle:270deg'), 'full knob should render full sweep');
assert(elements.cv_in.innerHTML.includes('Bearamin CV amount'), 'CV7 label should clarify Bearamin CV ownership');
assert(elements.outputs.innerHTML.includes('Bearamin X'), 'output reference missing');
assert(elements.resonator_web.hidden === false, 'resonator web controls should be mode-specific and visible');
assert(elements.resonator_web.innerHTML.includes('capped at 95%'), 'safe feedback cap copy missing');
assert(!elements.resonator_web.innerHTML.includes('Particle dust'), 'particle drone option remains');
assert(elements.resonator_web.innerHTML.includes('CV5 wet control'), 'CV5 reuse source missing');
assert(elements.resonator_web.innerHTML.includes('>Pitch<'), 'Pitch web source missing');
assert(elements.resonator_web.innerHTML.includes('>Roll<'), 'Roll web source missing');
assert(elements.resonator_web.innerHTML.includes('>Spank<'), 'Spank web source missing');
assert(elements.resonator_web.innerHTML.includes('Active on DPT'), 'applied revision state missing');
assert(elements.resonator_web.innerHTML.includes('Modulation source'), 'modulation source label missing');
assert(elements.resonator_web.innerHTML.includes('Modulation depth attenuverter'), 'modulation attenuverter label missing');
assert(!elements.resonator_web.innerHTML.includes('Depth is an attenuverter'), 'redundant feedback depth explanation should be removed');
assert(!elements.resonator_web.innerHTML.includes('name="res_feedback_tap"'), 'feedback tap radio group remains');
assert(elements.resonator_web.innerHTML.includes('name="res_drone_waveform"'), 'drone voice radio group missing');
assert(elements.resonator_web.innerHTML.includes('id="resonator_reverb_detail"'), 'nested reverb detail missing');
assert(elements.resonator_web.innerHTML.includes('<details'), 'reverb detail should be collapsible');
assert(!elements.resonator_web.innerHTML.includes('class="reverb-detail" open'), 'reverb detail should start collapsed');
assert(!elements.resonator_web.innerHTML.includes('Nested under CV5'), 'redundant nested title should be removed');
assert(reverbLpDisplay(60) === '7.0 kHz', 'low-pass should display its equivalent cutoff');
const sympatheticControls=elements.cv_in.innerHTML;
renderPanel({...ringsStatus,dpt_mode:0});
assert(elements.mode_name.textContent==='Modal', 'Modal label missing');
assert(elements.cv_in.innerHTML===sympatheticControls, 'Modal mappings differ from Sympathetic Strings');
assert(elements.resonator_web.hidden===false, 'Modal must expose resonator web controls');
renderPanel({...ringsStatus,dpt_mode:2});
assert(elements.mode_name.textContent==='Karplus-Strong', 'Karplus-Strong label missing');
assert(elements.cv_in.innerHTML===sympatheticControls, 'Karplus-Strong mappings differ from Sympathetic Strings');
assert(elements.resonator_web.hidden===false, 'Karplus-Strong must expose resonator web controls');

const panelBeforeFocusedPoll = elements.cv_in.innerHTML;
document.activeElement = {tagName:'SELECT', type:'select-one'};
renderPanel({dpt_mode:0,dpt_knobs:[255,255,255,255,255,255,0,0]});
assert(elements.cv_in.innerHTML === panelBeforeFocusedPoll, 'focused dropdown must survive status polling');
document.activeElement = null;

elements.settings_save.onclick();
assert(elements.preset_modal.hidden === false && elements.save_panel.hidden === false, 'save should open its modal');
closePresets();
openPreset('load');
assert(elements.preset_modal.hidden === false && elements.load_panel.hidden === false, 'load should open its modal');
renderPresetList({presets:[{name:'resonator-room'},{name:'default'}]});
assert(elements.preset_list.innerHTML.indexOf('default') < elements.preset_list.innerHTML.indexOf('resonator-room'), 'default should be pinned first');
assert(elements.preset_list.innerHTML.includes('Reload default'), 'default needs its explicit reload action');
// The same UI operation saves default without a card, while a named save
// reports the server failure. No additional backend choice is introduced.
const savedFetch = global.fetch;
const saveRequests = [];
global.fetch = async (url, options) => {
  saveRequests.push({url, options});
  return {json:async()=>url.endsWith('name=default')
    ? {ok:true,sd_ready:false,filename:'default.json'}
    : {ok:false,sd_ready:false,error:'sd'}};
};
const settle = async () => { for(let i=0;i<8;i++) await Promise.resolve(); };
elements.preset_default_save.onclick();
await settle();
assert(saveRequests[0].url==='/presets/save?name=default', 'default must use the ordinary save route');
assert(saveRequests[0].options.method==='POST', 'default save must be POST');
assert(elements.preset_save_state.textContent==='saved default.json', 'default save must succeed without SD');
elements.preset_name.value='room';
elements.settings_save_sd.onclick();
await settle();
assert(saveRequests[1].url==='/presets/save?name=room', 'named save must use the same route');
assert(elements.preset_save_state.textContent==='save error', 'named save must expose a failed save without SD');
renderPresetList({ok:true,sd_ready:false,presets:[{name:'default'}]});
assert(elements.preset_list.innerHTML.includes('Reload default'), 'default must remain loadable without SD');
global.fetch = async () => ({json:async()=>({...ringsStatus,boot_control_sync_source:'matrix_runtime'})});
await poll();
assert(elements.status_rows.innerHTML.includes('Session retained'), 'sidecar-only restart must report the retained session');
global.fetch = savedFetch;
setView(true);
assert(elements.instrument_view.hidden === true && elements.system_view.hidden === false, 'System should be a separate surface');
assert(captureWorkerMock.url === '/capture-worker.js', 'capture worker must be embedded and same-origin');
assert(captureWorkerMock.messages.some(message => message.type === 'library'), 'System should request the detailed capture library');
startLog();
const startMessage=captureWorkerMock.messages.find(message => message.type === 'start');
assert(!!startMessage, 'Capture should start with one click');
assert(/^capture-\d{8}T\d{6}Z-[0-9a-f]{8}$/.test(startMessage.id), 'automatic session identity is invalid');
assert(/^bearamin-capture-\d{8}T\d{6}Z$/.test(startMessage.basename), 'automatic filename base is invalid');
assert(elements.log_start.disabled === true && elements.log_stop.disabled === false, 'one-click capture state is invalid');
renderCaptureStatus({stage:'capturing',elapsedMs:61000,bytes:4096,records:20,drops:3,crcErrors:0,sequenceErrors:0,counts:{matrix:10},device:{capacity_bytes:1048576,used_bytes:2048,high_water_bytes:4096,store_errors:0}});
assert(elements.capture_compact.textContent.includes('1:01'), 'Instrument capture duration is missing');
assert(elements.capture_compact.textContent.includes('3 dropped records'), 'Instrument drop warning is missing');
assert(elements.capture_metrics.innerHTML.includes('High water'), 'System capture telemetry is missing');
stopLog();
assert(captureWorkerMock.messages.some(message => message.type === 'stop'), 'Stop must be delegated to the capture worker');
})().catch(err => { console.error(err); process.exit(1); });
'''
    subprocess.run(["node", "-e", harness + "\n" + script + "\n" + assertions], check=True, cwd=PROJECT_ROOT)


if __name__ == "__main__":
    main()
