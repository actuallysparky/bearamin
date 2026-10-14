# Optional Full Bearamin OBS helper

Start the combined helper from this folder:

```sh
python3 bearamin_obs_bridge.py --device-host YOUR-SIDECAR-HOST
```

Use `--help` for the OBS URL, ports and mapping controls. Keep it running while OBS uses the local Browser Source. Full Bearamin sends BOV1 telemetry through its sidecar; this helper receives it and also serves the mapping view. `bearamin.lan` and `bearamin-c5.lan` are the default device names; choose your own hostname when needed. A local `logo.png` is optional.

If you want a Mac login launcher, `python3 make_launch_agent.py` generates a plist under `_local/` using your current Python and checkout paths. It does not install or start it. The checked-in plist is a template only.

For Lite and Virtual’s optional MIDI grid, use [the grid helper](../../../tools/obs-grid/).
