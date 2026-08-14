# Robot Soccer Cup — Team Plan (prep: Aug 13, tournament: Aug 14, 2026)

Goal: win the 1-on-1 bracket, and if eliminated, win a Redemption Cup challenge. This doc assigns roles for the 5-person team (2 CS, 2 EE, 1 BME), lays out today's prep (no physical robot until tomorrow at the venue) and tomorrow's tight 5-hour dev window, and points at the new starter code written tonight.

## 1. What we're actually optimizing for

| Event | Format | How you win | Prize (per team member) |
| --- | --- | --- | --- |
| **Main bracket** | 1v1, round of 32, single elimination, 5-min matches | Most goals; tie → most ball touches (kicks) | Winner: LEGO FIFA World Cup Trophy 43020 / Runner-up: LEGO Soccer Ball 43019 |
| **Redemption Cup** (eliminated teams only) | Pick one bonus challenge | Judged per-challenge | Varies — see §6 |

**Fouls** (know these — they directly shape the AI policy): Yellow Card for *cornering the opponent's robot against a wall* or *tipping it over*; Red Card = 2 Yellows. A yellow card resets both robots to starting position. **This means the policy must actively avoid ramming/cornering the opponent — that's coded in tonight (see §4).**

**Team color isn't fixed**: you're red or blue per game, and color "may change during each round of play" — the robot must read its color live (`hold_toggle()`), not assume it. This is why `soccer_policy.py` re-reads `hold_toggle()` every tick instead of caching it once at startup.

Tomorrow's official schedule:

| Time | Activity |
| --- | --- |
| 9:00 AM | Check-in, breakfast |
| 10:00 AM | Kickoff + quick Edge Impulse workshop |
| 11:00 AM | **Dev begins** |
| 4:00 PM | **Tournament kickoff**, Redemption Cup after elimination |
| 6:00 PM | Winner announced |

**You have 5 hours of dev time tomorrow (11 AM–4 PM) with the actual hardware.** Everything we can front-load tonight in software directly buys back minutes in that window — that's the whole point of tonight's push.

## 2. Where the repo already stood this morning

Already built (earlier commits): UNO Q firmware (`sketch/sketch.ino`) with motor/servo/buzzer/LED/ultrasonic/line-sensor control, Bridge RPCs, BOOT-button program-enable + 5s-hold team toggle; ESP32-S3 camera firmware with per-robot MJPEG stream; `python/robot_client.py` (`MiniAutoRobot` wrapper); `python/main.py` (hardware bring-up demo); `python/capture.py` (dataset-capture tool — this is how the 321-image Edge Impulse dataset shown in your screenshot got built). Full narrative in `README.md` and the `AI LA x Qualcomm Robot Soccer Cup_ Developer Journey Guide.pdf` in the repo root.

**Not built until tonight:** any code that actually plays soccer. `python/main.py` is a demo motion sequence, not a strategy. That gap is now closed (§4).

## 3. What got built tonight (software-only — no hardware available until tomorrow)

Since the robot isn't here today, everything below was written, `python3 -m py_compile`'d, and exercised with **hardware-free self-tests** (synthetic frames/detections/robots — run any file directly, e.g. `python3 python/soccer_policy.py`, to see it prove itself with zero equipment attached). They still need real-hardware validation tomorrow — treat them as a strong first draft, not a finished product.

| File | What it does | Self-test |
| --- | --- | --- |
| `python/wall_detector.py` | `WallSideDetector` — HSV color classifier for the README's "Wall / Field-Side Detection" concept (red/blue field tape → which end of the field the camera faces). Pure OpenCV/numpy, no ML. | `python3 python/wall_detector.py` — synthesizes red/blue/neutral test frames, asserts correct classification. |
| `python/ei_runner.py` | `ObjectDetector` — wraps the real Edge Impulse Linux SDK (`edge_impulse_linux.image.ImageImpulseRunner`) around your exported FOMO model (`goal`/`robot`/`soccer_ball`). Fails fast with an actionable message if the SDK or `.eim` file isn't there yet. | `python3 python/ei_runner.py [path-to-.eim]` — degrades gracefully (no crash) when there's no model, which is the current state. |
| `python/soccer_policy.py` | `SoccerPolicy.decide_and_act()` — the match-play brain. One bounded action per tick (Acquire → Infer → Validate → Decide → Act → Reobserve). Has a **hybrid ball tracker** (tracks the ball normally, briefly extrapolates/coasts through short occlusions instead of instantly panic-searching, then falls back to a search biased toward the last-known side), **possession-safe goal scanning** (won't push forward unless the opponent's goal is positively confirmed — protects the ball with small peeks instead of guessing), and **two-tier opponent contact** (only retreats for genuinely imminent collisions; ordinary contested closeness gets a sideways juke, not a surrender). | `python3 python/soccer_policy.py` — 15+ scenarios incl. tracking/coasting/lost, possession-safe scan + its grace-period timeout, own-goal avoidance, and both opponent-contact tiers. |
| `python/play_match.py` | Optional match-day `App.run(...)` entry point wiring camera + model + wall detector + policy together. **Does not touch `app.yaml`/`main.py`** — switch to it deliberately once validated on hardware. | Degrades to a clean one-line message off-device. |
| `python/celebration.py` | Redemption Cup **Best Goal Celebration**: deliberately **cute, not cool** — a bashful pause, a small wiggle (not a big spin), a happy hop, a giggly uneven buzzer rhythm, and a soft pastel color blush (falls back to gentle `led()` blinking if `rgb()` isn't flashed yet). | `python3 python/celebration.py` — asserts the exact call sequence, pastel palette order, bounded/gentle pulse lengths, `stop()` on exception. |
| `python/precision_course.py` | Redemption Cup **Precision Course**: line-follow loop using the 4-channel line sensor, one bounded correction per tick. | `python3 python/precision_course.py` — scripted sensor sequences for every branch (steer left/right/forward, lost line, bad read, intersection hold, mid-move stop, timeout). |
| `python/trick_shot.py` | Redemption Cup **Trick Shot Challenge**: `TrickShotPolicy` composes `SoccerPolicy` and adds one ultrasonic obstacle-dodge branch in front of it. | `python3 python/trick_shot.py` — dodges a real obstacle, alternates dodge direction, correctly does *not* dodge when a close/centered ball explains the reading, delegates everything else unchanged. |
| `python/fastest_ball_detection.py` | Redemption Cup **Fastest Ball Detection**: minimal latency-focused "see ball, move, done" loop, deliberately separate from `SoccerPolicy` (no goal/opponent logic to pay for). | `python3 python/fastest_ball_detection.py` — exactly one `drive()` call, sane elapsed time, timeout path never drives blind. |
| `python/sim_match.py` | Not a challenge — a multi-tick integration storyboard that runs one `SoccerPolicy` instance through a scripted ~10-tick match (lose ball → find it → foul-avoidance interrupt → own-goal fake-out → score), catching cross-tick bugs the per-branch tests can't. | `python3 python/sim_match.py` |
| `python/diagnostics.py` | **Run this first tomorrow.** One-command morning bring-up check: robot health/sensors, camera stream, live wall-tape HSV calibration read, model load + one live inference — consolidates 5 separate manual checks into one. Every check is independent, so one failure doesn't block the rest. | `python3 python/diagnostics.py --model models/soccer-pico-160.eim` |

Also added tonight (small, low-risk, done directly rather than via draft code):
- **`robot.rgb(r, g, b)`** — new method on `MiniAutoRobot`, backed by a new `rgb` Bridge RPC in `sketch/sketch.ino` (mirrors the firmware's already-working serial `rgb r g b` command exactly). Lets the celebration routine show real colors instead of just white/off. **This needs a firmware reflash tomorrow to take effect** — until then, `celebration.py` automatically falls back to plain LED blinking (it checks `hasattr(robot, "rgb")` first).
- Verified with `python3 .agents/skills/uno-q-miniauto/scripts/check_robot_contract.py --strict` → **PASS**, and `python3 -m py_compile python/*.py` → clean, across the whole tree including the new files.

### The three pieces of strategy worth understanding before you touch anything tomorrow

Full reasoning for all three is in `python/soccer_policy.py`'s module docstring — read it before tuning constants.

1. **Own-goal disambiguation.** The object-detection model has **one generic `goal` class** — it cannot tell your goal from the opponent's by itself. Whenever a `goal` box is seen, `soccer_policy.py` also runs `wall_detector` on that *same* frame; whichever colored tape dominates tells you which end of the field you're facing, and comparing that to `hold_toggle()` tells you whether the goal in view is **yours** or the **opponent's**.
2. **Possession-safe scanning (new).** The policy no longer defaults to pushing forward when it *can't* tell whose goal is ahead. It only commits to a full-speed push once the goal is **positively confirmed as the opponent's**. If it's unconfirmed (no goal visible, or the wall tape isn't), it protects the ball with small bounded "peek" turns instead of guessing — for a bounded grace period (`POSSESSION_SCAN_GRACE_TICKS`, currently 6 ticks ≈ 1.2–1.5s) before cautiously proceeding anyway rather than stalling forever. **How patient to be there is a real risk-tolerance call — decide it with the team tomorrow, don't just trust the default.**
3. **Two-tier opponent contact (new).** The actual foul is *cornering against a wall* or *tipping over* — not mere proximity. The policy only retreats for genuinely imminent contact (`COLLISION_IMMINENT_MM`, tight); ordinary contested closeness gets a sideways juke around the opponent instead of a retreat, so the other team can't force us to keep surrendering the ball just by camping nearby.

Also new tonight: a **hybrid ball tracker** (`_BallTracker` in `soccer_policy.py`) — a lightweight EMA-smoothed velocity estimate, not a full Kalman filter (FOMO's 96×96 grid output is too coarse for that to pay off). While the ball is visible it tracks normally; for a few ticks after it disappears (`COAST_TICKS`) it keeps steering toward the extrapolated position instead of instantly panic-searching; only after that window expires does it fall back to search — seeded toward whichever side the ball was last seen on, not a blind coin flip. This mirrors how RoboCup Junior Soccer teams handle brief ball occlusion (see sources below).

**Three more additions, added after the model tuning was done — structural policy logic, not numeric tuning, so these were reasoned through rather than measured:**

- **Session state reset (`SoccerPolicy.reset()`).** A real bug, not a hypothetical: the policy is a long-lived object across the whole match, but a Yellow Card literally resets both robots and the ball to starting position. Without a reset, the ball tracker's remembered velocity/position, the search bias, and the possession-scan/push counters all survive a restart describing a world that no longer exists — e.g. confidently "coasting" toward where the ball used to be. `play_match.py` now calls `policy.reset()` at the start of every fresh BOOT-enabled session (every re-enable, whether from a card, a ref pause, or just practice).
- **De-wedge safety net (`DEWEDGE_PUSH_TICKS`).** There's no reliable way to detect "the robot is stuck pushing the ball against a wall" with this sensor suite — the ultrasonic reading to a ball being pushed successfully looks identical to one wedged motionless, and there's no wheel encoder to tell translation from a stall. Instead of detecting stuck-ness, the policy inserts one cheap sideways nudge after `DEWEDGE_PUSH_TICKS` (currently 10) *consecutive* confirmed pushes toward a confirmed opponent goal, then resumes. Costs little against a normal fast push, cheap insurance against wasting a whole match stalled against a wall.
- **Escalating search (`SEARCH_ESCALATE_AFTER_TICKS`).** Once genuinely lost (past the coast window) for a while, the search now mixes in an occasional strafe instead of only rotating in place forever, covering more of the frame. Lower-confidence value on a field this small — plain rotation will probably reacquire the ball fast regardless — but costs nothing when it isn't needed.

All three constants (`DEWEDGE_PUSH_TICKS`, `SEARCH_ESCALATE_AFTER_TICKS`, `SEARCH_WIDEN_EVERY_TICKS`) are unvalidated guesses like everything else numeric in this file — tune once you can see how long a real successful push or a real search actually takes.

**All of the numeric constants above are still tonight's best guesses, unvalidated against real hardware** — `python3 python/sim_match.py` proves the *logic* is internally consistent across a multi-tick match, not that the numbers are right for your actual field/opponents.

Sources consulted for the tracking design: [RoboCup Junior ball tracker (GitHub)](https://github.com/aul12/ROBOT), [Ball Detection and Tracking with Different Embedded Systems in the RoboCup Soccer context](https://www.researchgate.net/publication/376243587_Ball_Detection_and_Tracking_with_Different_Embedded_Systems_in_the_RoboCup_Soccer_context), [Real-time Localization of a Soccer Ball from a Single Camera](https://arxiv.org/pdf/2506.07981).

## 3.5 Model tuning results (Edge Impulse, done overnight) — full sweep, for context

Trained on **YOLO-Pro** (not FOMO — a deliberate switch, see reasoning below), swept model size and input resolution across six configurations:

| Config | mAP@50 | mAP@75 | Precision | float32 latency (Arduino UNO Q) |
| --- | --- | --- | --- | --- |
| small (6.9M), 96×96 | 0.79 | 0.25 | 82.8% | 154 ms |
| **nano (2.4M), 192×192 — saved as `soccer-nano-192.eim`** | **0.94** | **0.58** | **90.1%** | **118 ms** |
| **pico (682K), 160×160 — saved as `soccer-pico-160.eim`** | **0.91** | **0.46** | **88.8%** | **26 ms** |
| nano (2.4M), 160×160 | 0.90 | 0.43 | 87.2% | ~88 ms (est.) |
| pico (682K), 192×192 | 0.90 | 0.48 | 89.8% | 47 ms |

**Two non-dominated candidates survive**, both downloaded: nano/192 (best accuracy) and pico/160 (best speed, 4.5× faster than nano/192 for a ~3-point mAP@50 cost). Nothing else in the sweep beats both on its own axis, so the search is closed — **don't run more Edge Impulse experiments unless real hardware testing tomorrow specifically motivates one.**

**Why YOLO-Pro instead of FOMO** (the architecture README.md's "Model Import" section documents): FOMO predicts fixed-size grid-cell boxes, not real bounding boxes — its reported width/height don't scale with how close an object actually is. `soccer_policy.py`'s speed-scaling and `trick_shot.py`'s ball-vs-obstacle disambiguation both depend on real bounding-box size as a proximity signal, so YOLO-Pro's genuine variable-size boxes are a better fit for this codebase, not just a random choice.

**Why float32 beats int8 on this specific model/chip** (verified 4 separate times, not a fluke): unclear exactly why, but consistent — the Deployment tab's Arduino-UNO-Q-specific profiler showed Unoptimized (float32) meaningfully faster than Quantized (int8) in every single config tested. Accuracy-wise this costs nothing (full precision is never *less* accurate than its quantized version). If you retrain again for any reason, check both anyway before assuming the pattern holds forever.

**Which one to actually use in the match is still an open, real decision** — pico/160's 26ms is comfortably invisible against the ~150-250ms `robot.drive()` pulse durations already baked into every tick, which matters a lot on the tiny field from tonight's demo footage; nano/192's higher accuracy matters more if false negatives (missing the ball/goal entirely) turn out to be the bigger practical problem once you're actually watching it play. **Test both for real tomorrow — `python3 python/diagnostics.py --model <path>` is the fastest way to sanity-check either one loads and detects correctly before committing to it for a match.**

## 4. Role assignments

**We're at the venue now — robots not handed out yet.** Overnight model training and code work are both done; what's left is hardware bring-up, integration, and live tuning, all of which genuinely needs the physical robot. Assignments below are real, not placeholders.

### Phyo — Policy & Integration Lead
Deepest context on the redesigned `soccer_policy.py` (hybrid ball tracking, possession-safe scanning, two-tier opponent contact, de-wedge/search-escalation) and the overnight Edge Impulse tuning — owns the piece that needs that context most.
- **Right now:** handed off the Arduino IDE / esp32 core setup to Ken so you're free for this. Do a final skim of `python/soccer_policy.py`'s module docstring and `TOURNAMENT_PLAN.md` §3.5/§7 so the tunable constants (`CENTER_DEADZONE_FRAC`, `APPROACH_SPEED`, `COAST_TICKS`, `POSSESSION_SCAN_GRACE_TICKS`, `CONTESTED_MM`, `COLLISION_IMMINENT_MM`, `DEWEDGE_PUSH_TICKS`) are fresh going into live tuning.
- **Once the robot's up (after Ken's Stage 0-2, David's Stage 3-4):** wire `ei_runner.ObjectDetector` + `wall_detector.WallSideDetector` + `soccer_policy.SoccerPolicy` together via `play_match.py`, wheels-raised dry run first, confirm detections print sane values. Then wheels-down: tune speeds/deadzone, **deliberately test the own-goal-avoidance branch** (ball near your own colored wall — confirm the robot peels off, doesn't push), and the opponent juke-vs-retreat split. Own the `app.yaml` switch from `main.py` → `play_match.py` once validated — not before.

### Ken (EE) — Firmware & Hardware Bring-up Lead
- **Right now:** finish the esp32 core install (2.0.11, in progress) and pre-read `sketch/sketch.ino`, including the new `rpcRgb` Bridge RPC. If the kit's physically available under the signed Loaner Agreement, start a battery charge cycle.
- **Once the robot's in hand:** own Stage 0 (Safety) → Stage 2 (Control) from the PDF guide's bring-up table — wheels-raised power-on, build+flash `sketch.ino`, confirm `health()` reports `bridge:true serial:true`, run single-motor diagnostics, verify the BOOT-button red→yellow→green enable sequence and the 5-second team-toggle hold. Then flash the ESP32-S3 camera per `README.md`'s exact board/PSRAM/flash/partition/USB-CDC/flash-mode settings table.

### David (EE) — Vision & Sensor Calibration Lead
- **Right now:** read `python/wall_detector.py`. Field tape colors are confirmed real (good) — the HSV thresholds (`sat_min=120`, `val_min=70`) are still copied from documented defaults and will need retuning against the actual tape and venue lighting; that's expected, not a bug. Also cross-check the I2C address map (ultrasonic `0x77`, line sensor `0x78`, camera BOOT-button `0x79`) against the physical kit once you have it, before anyone reflashes anything.
- **Once Ken's camera is up:** own Stage 3 (Sensing) → Stage 4 (Vision) — confirm ultrasonic/line sensor readings are valid (not `-1`/`false`), confirm the MJPEG stream loads, then run `python3 python/diagnostics.py` for a live `wall_detector.py` calibration read and retune `sat_min`/`val_min`/`min_coverage_pct` until it cleanly separates red/blue/neutral under venue lighting. Hand tuned constants to Phyo for integration.

### Victor (CS) — Model Verification & Redemption Cup Lead
- **Right now:** read `python/trick_shot.py`, `python/fastest_ball_detection.py`, `python/precision_course.py`, `python/celebration.py` — all four Redemption Cup challenges are done and self-tested, but none have run on real hardware. These are self-contained and well-documented, a good place to get oriented without needing the overnight context.
- **Once the robot's up:** run `python3 python/diagnostics.py --model models/soccer-nano-192.eim` and again with `soccer-pico-160.eim` (see §3.5 for what each trades off) — this is the first real test either model has ever had. Help decide which one the team plays with. Then own getting the four Redemption Cup challenges running for real: `trick_shot.py`'s `OBSTACLE_MM`/ball-explains-it thresholds are the riskiest untested guess in the whole codebase — start there.

### Phone (BME) — Strategy, Redemption Cup & Team Ops Lead
- **Right now:** own the Game Rules and Redemption Cup sections cold — you'll make real-time judgment calls today (which challenge to enter if eliminated, whether a card dispute needs pushback). Confirm the Loaner Agreement is signed/returned if it hasn't been already — it may gate getting the kit at all. Recommended Redemption Cup priority (see §6): **Precision Course** and **Best Goal Celebration** first (no model-accuracy dependency), **Fastest Ball Detection** second, **Trick Shot** last (hardest, riskiest heuristics).
- **Match day:** timekeeping, who operates the BOOT button and when, battery swap plan between matches, point of contact with the referee, and the real-time call on whether/which Redemption Cup challenge to enter after elimination. Also good positioned to help Victor judge the Redemption Cup dry runs — "does this actually look/work right" is a judgment call, not just a code check.

## 5. Suggested tomorrow timeline (tight — 5 dev hours total)

**Updated from the original plan: model training and code are both done overnight, not tomorrow's job anymore.** That frees real time in the 11 AM–4 PM window for what actually needs hands-on-hardware time — integration, calibration, and match rehearsal.

| Time | Focus | Who |
| --- | --- | --- |
| 9:00–9:30 | Check-in; unbox/inspect kit; charge battery if not already | Ken, David |
| 9:30–10:00 | `pip install opencv-python numpy edge_impulse_linux` on the robot's Linux side — do this before the workshop saturates venue Wi-Fi | Ken |
| 10:00–11:00 | EI workshop (mandatory) | Everyone |
| 11:00–11:30 | Flash `sketch.ino` + verify `health()`; flash/verify camera stream | Ken, David |
| 11:30–11:45 | Drop both `.eim` files (`soccer-nano-192.eim`, `soccer-pico-160.eim`) into `models/`; run **`python3 python/diagnostics.py`** — the first real, hardware-validated pass on health/sensors/camera/wall-tape/model, all in one command | Whole team, one laptop |
| 11:45–12:15 | A/B the two models directly via `diagnostics.py --model <path>` against a real ball/robot/goal in front of the camera; pick a starting one (swappable later via `EI_MODEL_PATH`) | Victor, Phyo |
| 12:15–1:00 | Wire `ei_runner` + `soccer_policy` + `play_match` on real hardware, wheels-raised dry run; live-tune `wall_detector.py`'s HSV thresholds against real field tape (see §3.5 — **confirm the tape actually exists on this field first**) | Phyo, David |
| 1:00–1:45 | Wheels-down open-area test; tune speeds/deadzone; **deliberately verify own-goal avoidance AND the opponent juke-vs-retreat split** (see §3's design notes) | Phyo, David |
| 1:45–2:30 | Redemption Cup dry runs on hardware: `celebration.py`, `precision_course.py`, `trick_shot.py`, `fastest_ball_detection.py` | Phone, whoever's free |
| 2:30–3:15 | Full mock 5-minute matches; practice yellow-card restart; confirm BOOT-button operator handoff | Everyone |
| 3:15–3:45 | Buffer; switch `app.yaml` to `play_match.py` once validated; final charge | Everyone |
| 3:45–4:00 | Final gear check | Everyone |
| 4:00 PM | **Tournament starts** | — |

If something's behind schedule at 1:00, cut scope in this order: Redemption Cup dry runs → own-goal/juke live verification (keep it in code, just skip the live drill) → mock matches. Do **not** cut the wheels-raised sanity checks — that's how you avoid a match-day surprise.

## 6. Redemption Cup — pick order if eliminated

**Update: all four prep-able challenges now have working, self-tested code** (`celebration.py`, `precision_course.py`, `trick_shot.py`, `fastest_ball_detection.py`). The priority order below is now about which is *most reliable on the day*, not which has code at all.

| Priority | Challenge | Why | Prize (each member) |
| --- | --- | --- | --- |
| 1 | **Best Goal Celebration** | No model/camera dependency at all — just motion+LED+buzzer timing. Lowest risk of anything going wrong live. | Govee Smart Light Bars |
| 1 | **Precision Course** | Only depends on the line sensor (already proven hardware), no camera/model dependency. | Fanttik E1 Max Precision Screwdriver |
| 2 | **Fastest Ball Detection** | Depends on the `soccer_ball` class being confident and fast — strong given the overnight tuning results, otherwise the clock keeps running past your comfort zone. | Nike x LEGO Soccer Ball |
| 3 | **Trick Shot Challenge** | Hardest: needs the model AND correctly threading the "is this an obstacle or just the ball" ambiguity in `trick_shot.py` (see §7 below — the thresholds are unvalidated judgment calls, retune with real obstacles tomorrow before trusting this one). | LEGO Trickshot 43021 |
| — | **Attendees' Choice** | Community-judged, can't prep for it specifically — decide live if it comes up | LEGO Umbreon vs Garchomp |

## 7. Known risks / things to double-check, not assume

- **⚠️ FIRST THING TOMORROW: confirm the actual tournament field has the red/blue wall tape.** Demo footage from the team (Aug 13) shows a small green-floored box field with **no colored tape on the walls** — but the Edge Impulse dataset's own labelled images (the "goal" samples) clearly show blue tape behind at least one goal, so tape evidently exists in *some* setup. The entire own-goal-avoidance system (`wall_detector.py` + the goal-side check in `soccer_policy.py`) is USELESS without it — if the real competition field has no tape, `goal_side` will be `UNKNOWN` on every single tick, and the policy will spend its whole `POSSESSION_SCAN_GRACE_TICKS` window peeking before cautiously pushing every time it's near a goal, with zero ability to actually tell which goal is which. **Ask the organizers or check the field the moment you arrive** — if there's genuinely no tape, tell me or re-derive a different disambiguation strategy (e.g. remembering starting orientation, since each match starts with a fixed side assignment) before relying on this system in a real match.
- **The field is TINY.** From the same demo footage, the whole field looks roughly coffee-table-sized — robots and the ball are large relative to the playing area, and everything (approach, contact, going out of frame) happens fast and close. This directly undercuts several of tonight's placeholder constants: `APPROACH_SPEED=150`, `CONTESTED_MM=200`, and `COLLISION_IMMINENT_MM=100` were picked with no real sense of scale, and a 200mm "contested" zone could be a huge fraction of the entire field width. **Start tomorrow's first tests at noticeably lower speeds than the current defaults** — it's much easier to speed a working policy up than to debug a robot that's already put a dent in the wall. Mecanum sideways strafing (`drive("left"/"right")`) is confirmed working in the footage, which the juke/possession-scan logic already leans on.
- **The Edge Impulse dataset was auto-labelled, not hand-labelled — spot check it.** The sample metadata shows `labeled_by: owlv2` (an automatic zero-shot object detector, not a person) with a text prompt like "a black and white soccer ball." Auto-labelling can miss or misplace boxes, especially on the motion-blurred frames visible in the dataset. Before trusting the model's metrics, spot-check a handful of boxes — particularly any blurry ones — and tighten/fix any that are off; the README's own "Model Import" instructions already say to keep boxes tight.
- **Venue lighting ≠ home lighting.** Both `wall_detector.py`'s HSV thresholds and the Edge Impulse model were tuned/trained on your existing captures — budget real time tomorrow to retune, don't assume either transfers directly.
- **Both `.eim` files are exported and downloaded, but neither has run on real hardware yet.** They're `edge_impulse_linux` binaries built specifically for the Arduino UNO Q's aarch64 Linux side — they won't run on anyone's laptop to pre-check, Mac or otherwise. The very first real test of either one is tomorrow via `python3 python/diagnostics.py`. Don't assume they work until that's actually happened.
- **`robot.rgb()` needs a firmware reflash** to actually change colors — until `sketch.ino` is reflashed tomorrow, `celebration.py` silently falls back to plain LED blink, which is fine but less impressive.
- **Own-goal avoidance depends on wall-tape visibility — updated behavior.** If the camera can't see the field tape (bad angle, glare) or there's no goal in view at all, `goal_side` is unresolved and the policy no longer defaults to pushing forward (it did in the first draft — flagged as risky and since fixed based on team feedback). It now does small "peek" scans to protect the ball while trying to resolve which goal is which, for a bounded grace period (`POSSESSION_SCAN_GRACE_TICKS`) before cautiously proceeding anyway. **`POSSESSION_SCAN_GRACE_TICKS` is a real risk-tolerance dial the team should own** — shorter means more stalling risk, longer means more time spent not advancing the ball. Pick a value together tomorrow once you can see how it feels on real hardware.
- **Opponent contact is now two-tier, not one soft threshold.** Ordinary contested closeness (`CONTESTED_MM`) jukes sideways instead of retreating — only genuinely imminent contact (`COLLISION_IMMINENT_MM`, tighter) triggers a full backoff. This was changed specifically because the original single-threshold version was exploitable: an opponent camping near the ball/us could force repeated retreats without ever committing a foul themselves. Both distance thresholds and both bbox-size thresholds are still unvalidated guesses — retune with real opponents tomorrow, and if anything, err toward less skittish, not more.
- **The hybrid ball tracker's `COAST_TICKS` window is a guess too.** Too short and it's barely different from the old instant-search behavior; too long and it'll confidently steer at empty space for too long after a real loss. ~3 ticks (well under a second) is the starting point — watch this specifically during tomorrow's wheels-down testing.
- **Team color can change every round** — don't hardcode red/blue anywhere; the code already re-reads `hold_toggle()` every tick, keep it that way.
- **`app.yaml` still points at `python/main.py`.** That's deliberate — it's your tested fallback if `play_match.py` isn't ready in time. Don't switch it until Phyo signs off.
- **`trick_shot.py`'s obstacle-vs-ball ambiguity is a guess.** The ultrasonic sensor can't distinguish "there's an obstacle in the way" from "I'm right up against the ball" — the code disambiguates by checking whether a centered, large ball explains the close reading, using thresholds (`OBSTACLE_MM=200`, centering/size fractions) picked with zero real sensor data. Retune these against the actual obstacles and course layout before the Trick Shot run, not during it.
- **A real bug got caught by writing a real test, not just a syntax check:** `precision_course.py` originally imported `ProgramStopped` *inside* the function body, which would have crashed the moment it actually ran on a laptop (or anywhere `robot_client`'s `arduino.app_utils` dependency is missing) — `python3 -m py_compile` alone never would have caught it, since imports inside a function body aren't executed at compile time. Fixed by moving the import to module scope with a graceful fallback. Lesson for tomorrow: a file compiling clean is not the same as it having been run.
