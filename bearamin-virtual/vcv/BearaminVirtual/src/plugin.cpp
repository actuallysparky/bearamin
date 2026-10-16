// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Bearamin contributors

#include "plugin.hpp"
Plugin* pluginInstance;
void init(Plugin* p) {
    pluginInstance = p;
    p->addModel(modelBearaminVirtual);
}
