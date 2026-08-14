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

Also added tonight (small, low-risk, done directly rather than via draft code):
- **`robot.rgb(r, g, b)`** — new method on `MiniAutoRobot`, backed by a new `rgb` Bridge RPC in `sketch/sketch.ino` (mirrors the firmware's already-working serial `rgb r g b` command exactly). Lets the celebration routine show real colors instead of just white/off. **This needs a firmware reflash tomorrow to take effect** — until then, `celebration.py` automatically falls back to plain LED blinking (it checks `hasattr(robot, "rgb")` first).
- Verified with `python3 .agents/skills/uno-q-miniauto/scripts/check_robot_contract.py --strict` → **PASS**, and `python3 -m py_compile python/*.py` → clean, across the whole tree including the new files.

### The three pieces of strategy worth understanding before you touch anything tomorrow

Full reasoning for all three is in `python/soccer_policy.py`'s module docstring — read it before tuning constants.

1. **Own-goal disambiguation.** The object-detection model has **one generic `goal` class** — it cannot tell your goal from the opponent's by itself. Whenever a `goal` box is seen, `soccer_policy.py` also runs `wall_detector` on that *same* frame; whichever colored tape dominates tells you which end of the field you're facing, and comparing that to `hold_toggle()` tells you whether the goal in view is **yours** or the **opponent's**.
2. **Possession-safe scanning (new).** The policy no longer defaults to pushing forward when it *can't* tell whose goal is ahead. It only commits to a full-speed push once the goal is **positively confirmed as the opponent's**. If it's unconfirmed (no goal visible, or the wall tape isn't), it protects the ball with small bounded "peek" turns instead of guessing — for a bounded grace period (`POSSESSION_SCAN_GRACE_TICKS`, currently 6 ticks ≈ 1.2–1.5s) before cautiously proceeding anyway rather than stalling forever. **How patient to be there is a real risk-tolerance call — decide it with the team tomorrow, don't just trust the default.**
3. **Two-tier opponent contact (new).** The actual foul is *cornering against a wall* or *tipping over* — not mere proximity. The policy only retreats for genuinely imminent contact (`COLLISION_IMMINENT_MM`, tight); ordinary contested closeness gets a sideways juke around the opponent instead of a retreat, so the other team can't force us to keep surrendering the ball just by camping nearby.

Also new tonight: a **hybrid ball tracker** (`_BallTracker` in `soccer_policy.py`) — a lightweight EMA-smoothed velocity estimate, not a full Kalman filter (FOMO's 96×96 grid output is too coarse for that to pay off). While the ball is visible it tracks normally; for a few ticks after it disappears (`COAST_TICKS`) it keeps steering toward the extrapolated position instead of instantly panic-searching; only after that window expires does it fall back to search — seeded toward whichever side the ball was last seen on, not a blind coin flip. This mirrors how RoboCup Junior Soccer teams handle brief ball occlusion (see sources below).

**All of the numeric constants above are still tonight's best guesses, unvalidated against real hardware** — `python3 python/sim_match.py` proves the *logic* is internally consistent across a multi-tick match, not that the numbers are right for your actual field/opponents.

Sources consulted for the tracking design: [RoboCup Junior ball tracker (GitHub)](https://github.com/aul12/ROBOT), [Ball Detection and Tracking with Different Embedded Systems in the RoboCup Soccer context](https://www.researchgate.net/publication/376243587_Ball_Detection_and_Tracking_with_Different_Embedded_Systems_in_the_RoboCup_Soccer_context), [Real-time Localization of a Soccer Ball from a Single Camera](https://arxiv.org/pdf/2506.07981).

## 4. Role assignments

You gave me "2 CS majors, 2 EE majors, 1 BME major" with no names — assignments below are by skill-fit; swap freely, this is a starting point, not a mandate. Each role has a **today** (software/planning only) and **tomorrow** (hardware) column.

### CS #1 — Model / Edge Impulse Lead
- **Step zero — this is browser-only and needs zero hardware, do it right now:** the public link (`studio.edgeimpulse.com/public/1085406/live`) is a **read-only** view someone else owns — notice the "Clone this project" button. Click it (log into your own free Edge Impulse account first if needed) to get your own **editable copy** with the 321 images and the already-trained impulse. You can't add data, retrain, or export from the public view.
- **Then, in your cloned copy:** (1) Data acquisition tab — check the class balance (how many `goal`/`robot`/`soccer_ball` boxes exist); (2) spot-check a handful of samples, especially blurry ones — the dataset was auto-labelled by a model (`owlv2`), not a person, so some boxes may be loose or wrong; (3) Model testing tab — check **per-class** precision/recall, not just the aggregate 92.8%, since FOMO models often have one weak class dragging the average up; (4) add labeled images covering edge cases: ball partially hidden behind/under the robot, ball right at the goal mouth, robot at varying distance/angle, different lighting, and a few background-only frames (matches `capture.py`'s `empty` label — confirm those negatives are actually in the dataset, FOMO benefits a lot from them); (5) Retrain model tab — rerun training (150–180 cycles, LR 0.001, per README), re-check per-class metrics, aim for >90% on each class individually. "Live classification" lets you test against any photo you upload or a webcam, without the robot.
- **Tonight, once satisfied:** export **Deployment target: Linux (aarch64)**, download the `.eim`, and get it onto a USB stick / shared drive the team can access tomorrow without depending on venue Wi-Fi. Name it to match `play_match.py`'s default: `models/soccer-linux-aarch64.eim` (or set `EI_MODEL_PATH` at runtime to whatever you actually name it).
- **Tomorrow:** if venue lighting differs noticeably from your capture setup, use `python/capture.py` (already in the repo) to grab a quick batch of venue-lit images and do one fast retrain/re-export before the model gets locked in for matches.

### CS #2 — Policy & Integration Lead
- **Today:** Read `python/soccer_policy.py`, `python/play_match.py`, and `python/ei_runner.py` end to end — you own tuning and hardware integration tomorrow, so understand every branch now, not under time pressure later. Note the tunable constants at the top of `soccer_policy.py` (`CENTER_DEADZONE_FRAC`, `APPROACH_SPEED`, `COAST_TICKS`, `POSSESSION_SCAN_GRACE_TICKS`, `CONTESTED_MM`, `COLLISION_IMMINENT_MM`, etc.) — these were reasoned defaults, not measured ones, and §7 below explains why the field being much smaller than expected makes several of them suspect. Flag which look wrong for your actual field size/camera FOV so you're ready to retune fast tomorrow.
- **Tomorrow:** first hardware pass — wire `ei_runner.ObjectDetector` to the real exported model, run `play_match.py` with wheels raised, confirm detections look sane (print them). Then wheels-down in an open area, tune speeds/deadzone, and **deliberately test the own-goal-avoidance branch** (put the ball near your own colored wall and confirm the robot peels off instead of pushing it in). Own the `app.yaml` switch from `main.py` → `play_match.py` once it's validated — don't switch it before then, `main.py` is the fallback bring-up path.

### EE #1 — Firmware & Hardware Bring-up Lead
- **Today:** Review `sketch/sketch.ino`, including tonight's new `rpcRgb` Bridge RPC (small, mirrors the existing serial `rgb` handler exactly). Pre-download and install, on whichever laptop flashes tomorrow: Arduino IDE, **esp32 core 2.0.11 specifically** (not 3.x — README flags GC2145 init failures on 3.x), and the CH34x serial driver if on Windows — don't rely on venue Wi-Fi for this tomorrow morning. Print or save the "Pre-run checkpoint" and hardware bring-up stage table from the PDF guide (`AI LA x Qualcomm Robot Soccer Cup_ Developer Journey Guide.pdf`, "Journey at a glance" section) as your tomorrow-morning checklist. If the kit is physically accessible tonight under the Loaner Agreement, do a battery charge cycle now.
- **Tomorrow:** own Stage 0 (Safety) → Stage 2 (Control) from the guide's bring-up table: wheels-raised power-on, build+flash `sketch.ino`, confirm `health()` reports `bridge:true serial:true`, run single-motor diagnostics, verify the BOOT-button red→yellow→green enable sequence and the 5-second team-toggle hold.

### EE #2 — Vision & Sensor Calibration Lead
- **Today:** Read `python/wall_detector.py`. The HSV thresholds (`sat_min=120`, `val_min=70`, hue ranges from the README) are copied from the documented defaults — they *will* need retuning against your actual field tape and venue lighting; that's normal, not a bug. Prep the calibration procedure now: the file's `diagnostic_line()` output (`red=X% blue=Y% side=...`) is designed to be printed continuously for a few seconds at startup for exactly this tuning. Also cross-check the I2C address map (ultrasonic `0x77`, line sensor `0x78`, camera BOOT-button `0x79`) against the physical kit before anyone reflashes anything.
- **Tomorrow:** own Stage 3 (Sensing) → Stage 4 (Vision): confirm ultrasonic/line sensor readings are valid (not `-1`/`false`), confirm the MJPEG stream loads at `http://192.168.5.1:81/stream`, then run `wall_detector.py`'s logic live against the real field tape and retune `sat_min`/`val_min`/`min_coverage_pct` until `diagnostic_line()` cleanly separates red/blue/neutral under venue lighting. Hand tuned constants to CS #2 for integration.

### BME — Strategy, Redemption Cup & Team Ops Lead
- **Today:** Own the Game Rules and Redemption Cup sections cold — you're the one who'll make real-time judgment calls tomorrow (which challenge to enter if eliminated, whether a card dispute needs pushback). Read `python/celebration.py` and `python/precision_course.py` end to end (they're short) and sanity-check the tunables against what's likely to actually look good / score well — you don't need to code, just judge and request retuning. Recommended Redemption Cup priority given what's realistic to have working (see §6 for reasoning): **Precision Course** and **Best Goal Celebration** first (both are done tonight and don't depend on model accuracy), **Fastest Ball Detection** second (depends on the model being solid), **Trick Shot** last (hardest — needs both model accuracy and finer motion control). Confirm the Team Lead has signed and returned the Loaner Agreement so you can take the kit home if that's still open tonight. After every code merge today and tomorrow, run the two-command integration gate yourself: `python3 -m py_compile python/*.py && python3 .agents/skills/uno-q-miniauto/scripts/check_robot_contract.py --strict`.
- **Tomorrow:** match-day ops — timekeeping, who operates the BOOT button and when, battery swap plan between matches, point of contact with the referee, and the real-time call on whether/which Redemption Cup challenge to enter after elimination.

## 5. Suggested tomorrow timeline (tight — 5 dev hours total)

| Time | Focus | Who |
| --- | --- | --- |
| 9:00–10:00 | Check-in; unbox/inspect kit; charge battery if not already | EE #1, EE #2 |
| 10:00–11:00 | EI workshop (mandatory) — listen for anything that changes the export/deploy steps | Everyone, CS #1 leads |
| 11:00–11:45 | Flash `sketch.ino` + verify `health()`; flash/verify camera stream; finalize + export model with any workshop guidance; run the static integration gate on any last-minute changes | EE #1 / EE #2 / CS #1 / BME |
| 11:45–12:30 | Sensor validation (wheels raised); drop the `.eim` into `models/`; live-tune `wall_detector.py` against real field tape | EE #2 |
| 12:30–1:15 | Wire `ei_runner` + `soccer_policy` + `play_match` on real hardware, wheels-raised dry run | CS #2 |
| 1:15–2:00 | Wheels-down open-area test; tune speeds/deadzone; **deliberately verify own-goal avoidance** | CS #2, EE #2 |
| 2:00–2:45 | Redemption Cup dry runs (`celebration.py`, `precision_course.py`) on hardware | BME, whoever's free |
| 2:45–3:30 | Full mock 5-minute matches; practice yellow-card restart; confirm BOOT-button operator handoff | Everyone |
| 3:30–4:00 | Buffer; switch `app.yaml` to `play_match.py` once validated; final charge | Everyone |
| 4:00 PM | **Tournament starts** | — |

If something's behind schedule at 1:15, cut scope in this order: Redemption Cup polish → own-goal-avoidance live verification (keep it in code, just skip the live drill) → mock matches. Do **not** cut the wheels-raised sanity checks — that's how you avoid a match-day surprise.

## 6. Redemption Cup — pick order if eliminated

**Update: all four prep-able challenges now have working, self-tested code** (`celebration.py`, `precision_course.py`, `trick_shot.py`, `fastest_ball_detection.py`). The priority order below is now about which is *most reliable on the day*, not which has code at all.

| Priority | Challenge | Why | Prize (each member) |
| --- | --- | --- | --- |
| 1 | **Best Goal Celebration** | No model/camera dependency at all — just motion+LED+buzzer timing. Lowest risk of anything going wrong live. | Govee Smart Light Bars |
| 1 | **Precision Course** | Only depends on the line sensor (already proven hardware), no camera/model dependency. | Fanttik E1 Max Precision Screwdriver |
| 2 | **Fastest Ball Detection** | Depends on the model's `soccer_ball` class being confident and fast — strong if CS #1's model is solid, otherwise the clock keeps running past your comfort zone. | Nike x LEGO Soccer Ball |
| 3 | **Trick Shot Challenge** | Hardest: needs the model AND correctly threading the "is this an obstacle or just the ball" ambiguity in `trick_shot.py` (see §7 below — the thresholds are unvalidated judgment calls, retune with real obstacles tomorrow before trusting this one). | LEGO Trickshot 43021 |
| — | **Attendees' Choice** | Community-judged, can't prep for it specifically — decide live if it comes up | LEGO Umbreon vs Garchomp |

## 7. Known risks / things to double-check, not assume

- **⚠️ FIRST THING TOMORROW: confirm the actual tournament field has the red/blue wall tape.** Demo footage from the team (Aug 13) shows a small green-floored box field with **no colored tape on the walls** — but the Edge Impulse dataset's own labelled images (the "goal" samples) clearly show blue tape behind at least one goal, so tape evidently exists in *some* setup. The entire own-goal-avoidance system (`wall_detector.py` + the goal-side check in `soccer_policy.py`) is USELESS without it — if the real competition field has no tape, `goal_side` will be `UNKNOWN` on every single tick, and the policy will spend its whole `POSSESSION_SCAN_GRACE_TICKS` window peeking before cautiously pushing every time it's near a goal, with zero ability to actually tell which goal is which. **Ask the organizers or check the field the moment you arrive** — if there's genuinely no tape, tell me or re-derive a different disambiguation strategy (e.g. remembering starting orientation, since each match starts with a fixed side assignment) before relying on this system in a real match.
- **The field is TINY.** From the same demo footage, the whole field looks roughly coffee-table-sized — robots and the ball are large relative to the playing area, and everything (approach, contact, going out of frame) happens fast and close. This directly undercuts several of tonight's placeholder constants: `APPROACH_SPEED=150`, `CONTESTED_MM=200`, and `COLLISION_IMMINENT_MM=100` were picked with no real sense of scale, and a 200mm "contested" zone could be a huge fraction of the entire field width. **Start tomorrow's first tests at noticeably lower speeds than the current defaults** — it's much easier to speed a working policy up than to debug a robot that's already put a dent in the wall. Mecanum sideways strafing (`drive("left"/"right")`) is confirmed working in the footage, which the juke/possession-scan logic already leans on.
- **The Edge Impulse dataset was auto-labelled, not hand-labelled — spot check it.** The sample metadata shows `labeled_by: owlv2` (an automatic zero-shot object detector, not a person) with a text prompt like "a black and white soccer ball." Auto-labelling can miss or misplace boxes, especially on the motion-blurred frames visible in the dataset. Before trusting the model's metrics, spot-check a handful of boxes — particularly any blurry ones — and tighten/fix any that are off; the README's own "Model Import" instructions already say to keep boxes tight.
- **Venue lighting ≠ home lighting.** Both `wall_detector.py`'s HSV thresholds and the Edge Impulse model were tuned/trained on your existing captures — budget real time tomorrow to retune, don't assume either transfers directly.
- **The `.eim` file doesn't exist yet.** `ei_runner.py` and `play_match.py` will refuse to run (with a clear message, not a crash) until CS #1 exports one into `models/`.
- **`robot.rgb()` needs a firmware reflash** to actually change colors — until `sketch.ino` is reflashed tomorrow, `celebration.py` silently falls back to plain LED blink, which is fine but less impressive.
- **Own-goal avoidance depends on wall-tape visibility — updated behavior.** If the camera can't see the field tape (bad angle, glare) or there's no goal in view at all, `goal_side` is unresolved and the policy no longer defaults to pushing forward (it did in the first draft — flagged as risky and since fixed based on team feedback). It now does small "peek" scans to protect the ball while trying to resolve which goal is which, for a bounded grace period (`POSSESSION_SCAN_GRACE_TICKS`) before cautiously proceeding anyway. **`POSSESSION_SCAN_GRACE_TICKS` is a real risk-tolerance dial the team should own** — shorter means more stalling risk, longer means more time spent not advancing the ball. Pick a value together tomorrow once you can see how it feels on real hardware.
- **Opponent contact is now two-tier, not one soft threshold.** Ordinary contested closeness (`CONTESTED_MM`) jukes sideways instead of retreating — only genuinely imminent contact (`COLLISION_IMMINENT_MM`, tighter) triggers a full backoff. This was changed specifically because the original single-threshold version was exploitable: an opponent camping near the ball/us could force repeated retreats without ever committing a foul themselves. Both distance thresholds and both bbox-size thresholds are still unvalidated guesses — retune with real opponents tomorrow, and if anything, err toward less skittish, not more.
- **The hybrid ball tracker's `COAST_TICKS` window is a guess too.** Too short and it's barely different from the old instant-search behavior; too long and it'll confidently steer at empty space for too long after a real loss. ~3 ticks (well under a second) is the starting point — watch this specifically during tomorrow's wheels-down testing.
- **Team color can change every round** — don't hardcode red/blue anywhere; the code already re-reads `hold_toggle()` every tick, keep it that way.
- **`app.yaml` still points at `python/main.py`.** That's deliberate — it's your tested fallback if `play_match.py` isn't ready in time. Don't switch it until CS #2 signs off.
- **`trick_shot.py`'s obstacle-vs-ball ambiguity is a guess.** The ultrasonic sensor can't distinguish "there's an obstacle in the way" from "I'm right up against the ball" — the code disambiguates by checking whether a centered, large ball explains the close reading, using thresholds (`OBSTACLE_MM=200`, centering/size fractions) picked with zero real sensor data. Retune these against the actual obstacles and course layout before the Trick Shot run, not during it.
- **A real bug got caught by writing a real test, not just a syntax check:** `precision_course.py` originally imported `ProgramStopped` *inside* the function body, which would have crashed the moment it actually ran on a laptop (or anywhere `robot_client`'s `arduino.app_utils` dependency is missing) — `python3 -m py_compile` alone never would have caught it, since imports inside a function body aren't executed at compile time. Fixed by moving the import to module scope with a graceful fallback. Lesson for tomorrow: a file compiling clean is not the same as it having been run.
