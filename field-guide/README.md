<!-- {"canonical_sha256": "5f09cacd0b34a581133058812d818f89d98f60b556b208da7d6345d45f3d00e4", "canonical_source": "engineering:Bearamin-publication/field-guide/README.md", "editing": "Edit the canonical Obsidian source; regenerate this copy.", "generated": true, "vault_commit": null} -->

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

With Virtual firmware 0.3.0.6 and Rack plugin 2.1.3, the Field Guide and Rack grids use the saved Near/Far distances. Near is full brightness, Far is the dim edge, and farther cells are inactive. Unsaved edits do not alter the live preview. CV processing is unchanged. Older Virtual firmware remains usable; Rack falls back to its previous display range when range metadata is absent.
