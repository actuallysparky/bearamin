# Optional OBS field grid (Mac)

Run one helper to put Lite or Virtual’s depth grid in an OBS Browser Source. OBS still needs this helper running to receive MIDI and serve the page.

Connect one Bearamin by USB, or connect Lite through Audio MIDI Setup’s Bluetooth window. From this folder:

```sh
./run-mac.sh
```

The first run creates a local Python environment and installs the pinned MIDI dependencies. Python 3.10 or newer is required. Add an OBS Browser Source with URL `http://127.0.0.1:8767/` and dimensions 512 × 512. Keep the terminal open; Ctrl-C stops it. The view is transparent, clears stale frames and subscribes to the device’s 5 Hz preview. It does not change settings or firmware. Avoid using Field Guide’s preview at the same time: closing either subscriber can stop the shared preview lease.

For multiple MIDI ports, run `./run-mac.sh --list`, then select exact names with `--input 'name' --output 'name'`. Unplugging the device stops the helper; reconnect and run it again.

## Other grids

The same page can show a generic 8 × 8 distance feed, without MIDI dependencies:

```sh
my-grid-producer | python3 bridge.py --stdin
```

Write one JSON object per line: `{"mm":[...]}` with 64 row-major cells, each a distance from 1 to 4000 mm or `null`. The helper listens only on your computer. It ends when the producer closes its output.

Run `python3 test_bridge.py` for the codec, bounds and stale-frame checks. This optional tool currently targets Mac. Full Bearamin’s sidecar overlay is in [the existing helper](../../bearamin/tools/obs_overlay/).
