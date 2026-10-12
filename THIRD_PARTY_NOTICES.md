# Third-party notices

LighthousePairing is licensed under the GNU General Public License v3.0 (see `LICENSE`).
The distributed executable statically links the following components, each under its own license:

| Component | Version | License | Copyright |
|---|---|---|---|
| [Dear ImGui](https://github.com/ocornut/imgui) | v1.92.9b-docking | MIT | Copyright (c) 2014-2026 Omar Cornut |
| [ImPlot](https://github.com/epezent/implot) | 851cdbd | MIT | Copyright (c) 2020 Evan Pezent |
| [FreeType](https://freetype.org) | 2.14.3 | FreeType License (FTL) | Portions of this software are copyright © 2026 The FreeType Project (https://freetype.org). All rights reserved. |
| [nlohmann/json](https://github.com/nlohmann/json) | 3.12.0 | MIT | Copyright (c) 2013-2025 Niels Lohmann |
| [doctest](https://github.com/doctest/doctest) (tests only) | 2.4.12 | MIT | Copyright (c) 2016-2023 Viktor Kirilov |
| [libsurvive](https://github.com/collabora/libsurvive) (ported code, see below) | master 2024 | MIT | Copyright (c) 2016-2018 <>< Charles Lohr, J. David Berger, Michael W. Turvey; OOTX decoder (C) 2017 Joshua Allen |

Full license texts: `third_party/imgui/LICENSE.txt`, `third_party/implot/LICENSE`,
`third_party/freetype/docs/FTL.TXT`, and the headers of `third_party/nlohmann/json.hpp` / `third_party/doctest/doctest.h`.

### libsurvive

The Lighthouse wire-format decoding and LH2 math in `src/tracking/` are ported (rewritten in C++) from libsurvive:

- `protocol/WatchmanDecoder.cpp` – watchman report layout, raw0 / raw1 light data (`driver_vive.c`)
- `protocol/Commands.h` – light mode switch command (`driver_vive.c`, `vive_switch_mode`)
- `lighthouse/Ootx.cpp` – OOTX bit stream decoder (`ootx_decoder.c`)
- `lighthouse/BaseStation.cpp` – OOTX v15 calibration layout, gen2 reprojection model (`ootx_decoder.c`, `survive_reproject_gen2.c`)
- `lighthouse/Lh2Processor.cpp` – rotor frequencies, sync / sweep to angle (`survive_process_gen2.c`)

libsurvive is distributed under the MIT License reproduced below.

## MIT License (Dear ImGui, ImPlot, nlohmann/json, doctest, libsurvive)

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

## FreeType

Used under the FreeType License (FTL), option 1 of FreeType's dual license.
The FTL is compatible with GPLv3.

"Valve", "Index", "SteamVR", "HTC VIVE", "Tundra" and "Pimax" are trademarks of their
respective owners. This project is not affiliated with or endorsed by any of them.
