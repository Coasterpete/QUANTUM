# Intermittent preview lag investigation â€” 2026-09-06

This investigation corrected stale simulation wall time after minimize/restore.
It did **not** eliminate all intermittent host/render stalls. No physics or
renderer optimization is justified by the measurements collected here.

## Reproduction and attribution

The real executable ran `--dev-preview-smoke smoke-tests/preview-transition.quantum
--repeat --duration 60 --output build/diagnostics/<run>` in Debug and Release.
The original fixture reached its legal open endpoint after 2.287 seconds, so the
initial non-repeating invocation failed its requested 60-second duration. The
small developer-only repeat option resets/replays only a reported Core boundary
intervention; availability/interpolation failures and other stops still fail.
The collector and frame IDs remain continuous across replays. The supplied
fixture is a 60 m straight profile despite its filename; it is not a demanding
curved-track benchmark.

Windows Computer Use (`@oai/sky`) successfully launched normal Debug QUANTUM,
opened Performance Telemetry, edited a temporary unsaved straight track to
6000 m, started playback, minimized and restored it, and captured screenshots.
Normal playback showed roughly 100 FPS, 3 steps and 1.134 ms physics CPU in an
observed frame. The restore screenshot showed a large rolling raw-delta spike;
it did not capture that exact frame's counters. The temporary document was
closed without saving. A separate repeating Release smoke run was manipulated
through Computer Use to capture the restore numerically. Attempted resize drags
did not visibly resize the viewport; these are not counted as resize verification.

### Uninterrupted baseline

Debug frame 1281 spent **64.5402 ms** waiting for the frame-slot fence, with
0.7818 ms physics, 1.2279 ms pre-simulation work and 65.2191 ms drawFrame.
Frame 1282 then received **67.3938 ms**, requested/executed **16/16** ticks,
and spent **6.2603 ms** in physics. Individual steps were
0.3535 / 0.3899 / 0.5845 ms min/average/max. Its fence wait was 0.0273 ms,
event pump 0.0110 ms, and pre-simulation work 1.1015 ms. No blocking tags
were set. Every retained Debug spike followed a substantial fence wait.
The largest consecutive catch-up streak was one: no physics catch-up spiral.

Release reproduced the same distinction. Frame 1283 had a **54.8263 ms fence
wait**, 0.2036 ms physics and 55.1782 ms drawFrame. Frame 1284 received
**56.0039 ms**, requested/executed **13/13**, and spent **0.8560 ms** in physics
(0.0561 / 0.0656 / 0.0862 ms per-step min/average/max). No blocking tags
were set. Release's much cheaper physics did not eliminate these stalls.

These are measured waits inside `vkWaitForFences`, not proof that GPU rendering
itself took that long. Display scheduling, GPU contention, driver latency and
host preemption cannot be separated by these CPU-side measurements. The wait
protects frame-slot resource reuse and was preserved. FIFO/VSync was preserved.

### Native minimize/restore baseline

Release frame 2131 was ordinary: 10.1920 ms input, 3 steps, 0.0700 ms physics,
9.2086 ms fence wait. Minimize/restore then prevented rendered/ImGui frames.
Frame 2132 received **10191.5588 ms**, with minimize, restore, focus-loss,
focus-regain and swapchain-recreation tags. It clamped to 250 ms and executed
**60/60** requested ticks, dropping 9942.7297 ms including the old accumulator.
Physics took **1.1320 ms**, with step min/avg/max 0.0182 / 0.0188 / 0.0336 ms.
Current event pump was 6.5703 ms and drawFrame 5.2408 ms. The next frame 2133
received 6.6374 ms and executed one tick: no feedback loop.

**Confirmed defect:** Application skips ImGui NewFrame while minimized, then
passes the first restored ImGui DeltaTime unchanged to SimulationPreview.
Time spent suspended is therefore turned into artificial playback ticks.

Eight later native-capture spikes had roughly 124 ms in the current event pump,
not in physics or the previous fence wait. For example frame 2759 received
133.7537 ms, event pump 123.7359 ms, requested/executed 33/33 and physics
0.6352 ms; the next frame received 2.5009 ms and executed zero ticks. There
were no identifying tags. The lower-level event-pump cause remains unexplained;
the interaction/capture environment may matter, but that is not established.

## Exact correction and safety

Application passes a timing-discontinuity flag only when the existing event
telemetry reports minimize or restore. These flags persist across skipped
minimized iterations. SimulationPreview keeps recording the raw delta, reports
that interval as discarded wall time, and executes no ticks for that update.
The fractional accumulator, committed state, interpolation alpha and render pose
remain intact. The following ordinary frame resumes from the same fractional
tick. No resources or ownership/lifetime rules changed.

This changes editor preview wall-clock synchronization after suspension only.
Core still uses exactly 1/240 second per tick. Ordinary input deltas, the 60-step
catch-up guard, geometry, tolerances, interpolation and Vulkan synchronization
are unchanged. No arbitrary catch-up budget was introduced. Focus-only,
resize, dialogs and untagged stalls were not given speculative suppression.

## Files changed by this investigation

- `editor/include/quantum/editor/SimulationPreview.hpp`: optional interruption
  argument and per-frame normal-boundary marker.
- `editor/src/SimulationPreview.cpp`: preserve state/remainder on interruption;
  mark genuine boundary stops for developer replay.
- `engine/src/Application.cpp`: connect retained minimize/restore flags;
  restart only genuine boundary completions in repeat smoke mode.
- `editor/include/quantum/editor/PreviewSmoke.hpp`: opt-in repeat flag and bounded
  following-frame spike context.
- `editor/src/PreviewSmoke.cpp`: parse repeat; serialize existing event-pump,
  previous physics and current renderer timings; retain the following frame.
- `editor/src/EditorUi.cpp`: discarded-time label now includes interruptions.
- `tests/SimulationPreviewTests.cpp`: interruption/remainder/pose regression and
  deterministic boundary replay checks; existing physics expectations retained.
- `tests/PreviewSmokeTests.cpp`: opt-in CLI and following-frame retention checks.
- `docs/architecture.md`: document editor suspension timing semantics.
- `docs/lag-investigation.md`: this report.

The workspace already contained uncommitted physics, renderer, telemetry and
smoke implementation work. Those changes were preserved. This investigation
made no Core or renderer implementation edits and did not commit.

## Validation and measurements

Detailed JSON/TXT reports and build/test logs are under `build/diagnostics/lag-*`.
Measurements include startup and fixture replay. Native averages/worst intervals
also include deliberately minimized time, so they are not useful measures of
active playback throughput. CPU timers measure elapsed time around CPU calls;
they do not distinguish execution from preemption and are not GPU timestamps.

Builds: `cmake --build build --config Debug --parallel 4` and the corresponding
Release command succeeded. Full `ctest --test-dir build -C <configuration>
--output-on-failure --parallel 4` passed **63/63 in Debug (390.70 s)** and
**63/63 in Release (39.54 s)**. This includes rollback, rigid-bogie, connector,
interpolation and simulation coverage. The new test initially needed forward
declarations for existing comparison helpers; that compile error was corrected
before the successful full builds. `git diff --check` passed.

The regression compares committed dynamics/poses exactly against a control
preview, injects a 10-second discontinuity after 2.5 ticks, verifies zero executed
ticks and unchanged render geometry, then confirms the next half-tick completes
the preserved fractional tick. Boundary replay reaches exactly the same endpoint
state and clears its completion marker on the next frame. Collector tests verify
bounded following-frame retention and the opt-in repeat argument.
