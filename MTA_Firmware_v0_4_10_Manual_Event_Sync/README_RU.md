# WANT!PAY!TAKE! — v0.4.10 Manual Event Sync (EXPERIMENTAL)

**Main sketch**: `MTA_Firmware_v0_4_10_Manual_Event_Sync.ino` in a folder of the SAME name.

## What changed
- Version headers, Serial startup label, and dedicated Arduino sketch folder: **v0.4.10**.
- Automatic event delivery is **OFF** (`MTA_ENABLE_EVENT_UPLOAD = false`).
- A new capital `S` command enables **one** manually requested transmission of the existing queued event `MTA-001-E0000000002`, type `LOAD`, product `DOBRY_COLA_025`, quantity 1, only if it is the sole event in the journal and local stock is 1.
- The guarded command requires IDLE, no motor motion, no active payment/fault/uncertainty, no unfinished `stockop`, valid clock, and available queues. No new LOAD event is created; local stock is unchanged.
- Manual transmission happens at most once per controller boot. No automatic retry after HTTP failures. On server `status=accepted` or `status=already_accepted` for the matching `event_id`, journal head is acknowledged and removed. Otherwise it remains for later investigation.
- HTTPS Heartbeat continues as before. v0.4.9 stock operation recovery remains.
- Server program, PaymentManager, GPIO and mechanical state logic not modified. Neither older sketch directories nor main branch changed.

## Safe staged testing
1. Download the experimental branch. Open the new v0.4.10 folder. Copy **locally** the correct `mta_secrets.h` from the HTTPS-working installation. Do not commit real secrets.
2. In Arduino IDE select **ESP32 Dev Module**, verify COM port, compile. **Compilation on actual Arduino IDE and device tests are NOT yet verified.**
3. ONLY if compilation succeeds, upload without "Erase All Flash" and check Serial Monitor 115200, **No Line Ending**. Motor and payments must remain inactive.
4. Confirm startup `Firmware v0.4.10 Manual Event Sync` and repeated `Heartbeat ... HTTP=200`.
5. Run `Q`, `I`, `L`: expect safe PaymentManager state, stock=1, and ONLY event `MTA-001-E0000000002` LOAD for DOBRY_COLA_025. If not exact, **STOP**.
6. On VPS independently verify stock=0 and event ID absent (read-only SQLite) and create a fresh SQLite backup. Do **not** change VPS data manually.
7. **Do not send `S` until explicitly coordinating the trial.** One `S` should enqueue one HTTPS request. Expected on success: `Event ... HTTP=200 confirmed=1`, then `[MTA EVT] CONFIRMED ... remaining=0`. On failure, no auto-retry; record the diagnostics before doing anything else.
8. Check VPS inventory=1 and events table includes the event exactly once. Local stock remains 1; NVS journal becomes 0. If network result ambiguous, check VPS first before resetting ESP32 or any retry.

## Scope and limitations
- This is a **diagnostic** single-event delivery, not production sync. It only handles the already-recorded LOAD #2 and will refuse other events.
- The manual-at-most-once flag is volatile (reset at reboot). Server-side event_id idempotency is required to prevent double stock changes if re-sent after reboot.
- The code validates `HTTP=200`, exact returned status and event_id in the FastAPI JSON response before local acknowledgement; no cryptographic application-level signed receipt.
- If a power failure happens during the network request, the local event can persist even though the server accepted it. This is expected: server must answer `already_accepted` on the next controlled attempt.
- We did NOT run forced-power-loss tests, real payment or physical dispensing tests. Do not use in commercial operation.
