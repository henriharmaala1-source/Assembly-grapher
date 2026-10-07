-- kestrel_osd.lua -- the companion computer's status on ArduPilot's OSD.
--
-- kestrel (the Pi) sends one STATUSTEXT line whenever its state changes:
--
--     SCRIPT LIVE VEL GO DOOR
--     |      |    |   |  `-- a script's current state
--     |      |    |   `----- the GO latch is on
--     |      |    `--------- how commands reach this FC in ITS current mode:
--     |      |               VEL velocity (GUIDED)   ATT attitude (GUIDED_NOGPS)
--     |      |               RC  sticks (ALT_HOLD/LOITER/POSHOLD)
--     |      |               AST assist   OFF nothing in this mode
--     |      |               PAR still reading parameters
--     |      `-------------- LIVE: commands are sent. DRY: computed, not sent
--     `--------------------- kestrel's mode (or RTH / LAND)
--
-- ArduPilot only LOGS a STATUSTEXT it receives; it never reaches the OSD. This
-- script re-issues it with gcs:send_text, which is what the OSD MESSAGE panel
-- draws: OSD_MSG_TIME seconds (10 by default), 26 characters, upper case,
-- scrolled if longer. It also watches kestrel's own HEARTBEAT and puts
-- "K LOST" up when it stops: the last line shown is then stale, and the
-- pilot should know that before trusting it.
--
-- INSTALL (onboard/docs/pi5-ardupilot-setup.md): SCR_ENABLE = 1, reboot, copy
-- this file to APM/scripts/ on the FC's SD card, reboot. OSD1_MESSAGE_EN = 1.
-- Needs a flight controller with scripting (2 MB flash: F7/H7; most 1 MB F405
-- builds leave it out).

local KESTREL_SYSID  = 255    -- MavlinkBackend's sysid (= MAV_GCS_SYSID / SYSID_MYGCS)
local KESTREL_COMPID = 191    -- MAV_COMP_ID_ONBOARD_COMPUTER, not a GCS's 190
local REFRESH_MS     = 8000   -- re-show before OSD_MSG_TIME hides it; 0 = on change only
local LOST_MS        = 3000   -- no kestrel heartbeat for this long = say so
local PREFIX         = "K "
local LOOP_MS        = 100

local MSG_HEARTBEAT, MSG_STATUSTEXT = 0, 253

mavlink:init(10, 2)           -- queue depth, number of message ids
mavlink:register_rx_msgid(MSG_HEARTBEAT)
mavlink:register_rx_msgid(MSG_STATUSTEXT)

-- The bindings hand over the C mavlink_message_t, not the wire bytes. Its
-- layout is the one ArduPilot's own modules/MAVLink/mavlink_msgs.lua decodes:
-- 291 bytes with a 1-byte sysid on older firmware, >= 298 with a 4-byte sysid
-- (MAVLink 2.1) on newer. Returns len, sysid, compid, msgid, payload offset.
local function header(msg)
  local len = string.unpack("<B", msg, 4)
  if #msg >= 298 then
    return len, string.unpack("<I4", msg, 8), string.unpack("<B", msg, 12),
           string.unpack("<I3", msg, 13), 16
  end
  return len, string.unpack("<B", msg, 8), string.unpack("<B", msg, 9),
         string.unpack("<I3", msg, 10), 13
end

-- STATUSTEXT: severity, then 50 chars of text, NUL-terminated only if
-- shorter. Only the first `len` payload bytes are the message; anything after
-- them in the structure is not ours to read.
local function statustext(msg, len, ofs)
  local last = ofs + math.min(len, 51) - 1
  local text = string.sub(msg, ofs + 1, last)
  local nul = string.find(text, "\0", 1, true)
  if nul then text = string.sub(text, 1, nul - 1) end
  return text
end

local text, shown_ms, hb_ms, lost = nil, 0, nil, false

local function show(severity, t, now)
  gcs:send_text(severity, PREFIX .. t)
  shown_ms = now
end

local function update()
  local now = millis():toint()
  for _ = 1, 10 do
    local msg = mavlink:receive_chan()
    if not msg then break end
    local len, sysid, compid, msgid, ofs = header(msg)
    if sysid == KESTREL_SYSID and compid == KESTREL_COMPID then
      if msgid == MSG_HEARTBEAT then
        hb_ms = now
        if lost then
          lost = false
          if text then show(6, text, now) end
        end
      elseif msgid == MSG_STATUSTEXT then
        local t = statustext(msg, len, ofs)
        if t ~= "" and t ~= text then
          text = t
          if not lost then show(6, t, now) end
        end
      end
    end
  end

  if hb_ms and not lost and now - hb_ms > LOST_MS then
    lost = true
    show(4, "LOST", now)             -- WARNING: the Pi stopped talking
  elseif REFRESH_MS > 0 and now - shown_ms >= REFRESH_MS then
    if lost then show(4, "LOST", now)
    elseif text then show(6, text, now) end
  end
  return update, LOOP_MS
end

-- A decode surprise on some firmware must not kill the OSD line for the
-- rest of the flight: report it once, keep going.
local reported = false
local function protected()
  local ok, a, b = pcall(update)
  if ok then return protected, b end
  if not reported then
    gcs:send_text(3, "kestrel_osd: " .. tostring(a))
    reported = true
  end
  return protected, 1000
end

return protected()
