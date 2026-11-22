#pragma once
#include <bearfield/bearfield.h>
#include "editor_contract.h"
#include "../common/bearamin_virtual_wire.h"

namespace virtual_editor {
inline bearfield::FeelProfile feelProfile(const Settings& settings) {
  auto profile = bearfield::BearCvV1();
  profile.minimumDepthMm = settings.near;
  profile.playableMaximumMm = settings.far;
  profile.fadeMaximumMm = settings.far + 150;
  profile.maximumSourceDepthMm = settings.far + 150;
  profile.presenceHoldMs = settings.hold;
  profile.minimumBlobCells = settings.cells;
  profile.spankMaximumDepthMm = settings.spankDepth;
  profile.spankMinimumApproachMmPerSecond = settings.spankSpeed;
  profile.spankRefractoryMs = settings.cooldown;
  profile.axisTauSeconds = profile.depthTauSeconds = settings.smoothing / 1000.0f;
  return profile;
}

inline bearfield::DepthFrame8x8 orient(const bearfield::DepthFrame8x8& raw,
                                     const Settings& settings) {
  bearfield::SensorMountConfig mount{};
  mount.rawToCanonical.rotation = static_cast<bearfield::QuarterTurns>(settings.rotation / 90);
  mount.rawToCanonical.mirrorX = settings.mirror;
  return bearfield::orientFrame(raw, mount);
}

inline bearamin_virtual::Gesture gesture(const bearfield::ProcessedFieldState& field,
                                        const Settings& settings) {
  bearamin_virtual::Gesture result{};
  result.x = mapAxis12(field.xNorm12, settings.curveX, settings.invertX);
  result.y = mapAxis12(field.yNorm12, settings.curveY, settings.invertY);
  result.z = mapAxis12(field.zNorm12, settings.curveZ, settings.invertZ);
  result.centroidX = field.xMilli;
  result.centroidY = field.yMilli;
  result.present = field.present;
  result.presence = field.present && settings.presenceEnabled;
  result.spank = field.present && field.spankHeld && settings.spankEnabled;
  result.rotation = settings.rotation / 90;
  result.mirror = settings.mirror;
  return result;
}
template <typename Send>
inline void performanceMidi(const bearfield::ProcessedFieldState& field,
                            const Settings& settings, bool& noteHeld, Send send) {
  const uint8_t channel = settings.channel - 1;
  send(0xb0 | channel, settings.ccX, field.present ? mapAxis(field.xNorm12,settings.curveX,settings.invertX) : 0);
  send(0xb0 | channel, settings.ccY, field.present ? mapAxis(field.yNorm12,settings.curveY,settings.invertY) : 0);
  send(0xb0 | channel, settings.ccZ, field.present ? mapAxis(field.zNorm12,settings.curveZ,settings.invertZ) : 0);
  if (settings.presenceEnabled) send(0xb0 | channel,settings.ccPresence,field.present ? 127 : 0);
  const bool held = field.present && settings.spankEnabled && field.spankHeld;
  if (held != noteHeld) {
    send((held ? 0x90 : 0x80) | channel,settings.note,held ? settings.velocity : 0);
    noteHeld = held;
  }
}
} // namespace virtual_editor
