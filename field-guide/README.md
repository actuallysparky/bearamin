<!-- {"canonical_sha256": "61e19bac39da691d9b0e438f004fd183fc96f0a1e50f768bd079579b9764155e", "canonical_source": "engineering:Bearamin-publication/field-guide/README.md", "editing": "Edit the canonical Obsidian source; regenerate this copy.", "generated": true, "vault_commit": null} -->

# Bearamin Field Guide

One friendly place for Bearamin Lite and Virtual settings and firmware.

Connect your device, choose **Connect**, and the guide identifies it. A current device leads to **Let's play**; an older device leads to **Update**. MIDI and gestures share one tab, while System contains connection help, diagnostics and firmware information. Each setting shows its current value and help with its default.

USB MIDI works with Lite and Virtual. Lite also connects through Bluetooth MIDI after connection in your computer's MIDI settings. Use a Web MIDI browser such as Chrome or Edge over HTTPS. Virtual's first installation uses UF2; later compatible updates use MIDI.

The public page offers only the latest firmware for each device. Full Bearamin keeps its sidecar web interface.

[Open the Field Guide](https://actuallysparky.github.io/bearamin/) · [Bearamin family](<../README.md>)

## Publish a build

Use the repository’s **Build and publish a beta** workflow and choose Lite, Virtual or Full. It builds the firmware, packages verified release assets, creates a beta release, and updates GitHub Pages automatically. Bump Virtual’s four-part firmware version before changing its bytes; an existing version cannot be overwritten. Changes to the guide and published releases also trigger the Pages workflow. Only the current payloads are placed on the website.


## Virtual settings in VCV Rack

Use Virtual firmware 0.3.0.5 or newer with the [Bearamin VCV plugin 2.1.2](https://github.com/actuallysparky/Bearamin-VCV/releases/tag/v2.1.2). Save your Field Guide changes to apply them to MIDI and Rack. New and existing Rack modules follow the saved device settings automatically. Range and Slew stay in Rack. Older plugins continue their own raw-depth processing.
